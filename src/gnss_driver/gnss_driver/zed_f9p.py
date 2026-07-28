"""Transport-agnostic NMEA driver for the u-blox ZED-F9P (ArduSimple RTK).

The ZED-F9P streams standard NMEA-0183 sentences out of the box (no receiver
reconfiguration needed). This module reads that stream through a
:class:`~gnss_driver.transport.Transport` and assembles complete position fixes
in engineering units (degrees, metres, m/s), so the ROS node only has to stamp
and publish them.

Sentences consumed (any talker id — ``GN`` for the multi-constellation
solution, or ``GP``/``GL``/``GA``/``GB`` per system):

* **GGA** — fix quality, latitude, longitude, altitude, #satellites, HDOP.
  This is the sentence that *completes* a fix; the others just annotate it.
  A GGA with empty position fields still yields a :class:`GnssFix`, flagged
  ``position_valid=False`` — see that class for why.
* **GST** — per-axis position error std-devs, turned into a diagonal
  covariance (this is what makes the fix's uncertainty trustworthy for fusion).
* **VTG** / **RMC** — speed and course over ground, turned into an ENU
  horizontal velocity.

The module is deliberately ROS-free (like ``bmi088.py``) so the parsing and unit
conversions can be unit-tested without ROS or hardware. Covariance-type values
mirror ``sensor_msgs/NavSatFix`` so the node can copy them straight through.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from gnss_driver.transport import Transport

# NavSatFix.position_covariance_type constants, duplicated here to keep this
# module import-free of ROS (values are fixed by the message definition).
COV_TYPE_UNKNOWN = 0
COV_TYPE_APPROXIMATED = 1
COV_TYPE_DIAGONAL_KNOWN = 2
COV_TYPE_KNOWN = 3

# GGA field 6 (fix quality) -> is this a usable fix, and is it RTK?
GGA_QUALITY_NO_FIX = 0
GGA_QUALITY_GPS = 1          # autonomous / SPS
GGA_QUALITY_DGPS = 2         # differential
GGA_QUALITY_RTK_FIXED = 4    # RTK integer-ambiguity fixed (cm-level)
GGA_QUALITY_RTK_FLOAT = 5    # RTK float
GGA_QUALITY_DEAD_RECKON = 6

KNOTS_TO_MPS = 0.514444
KMH_TO_MPS = 1.0 / 3.6


@dataclass
class GnssFix:
    """One assembled position fix, in units the ROS node can publish directly."""

    latitude: float                       # deg, WGS-84 (NaN if not position_valid)
    longitude: float                      # deg, WGS-84 (NaN if not position_valid)
    altitude: float                       # m, above WGS-84 ellipsoid (NaN if invalid)
    quality: int                          # GGA fix-quality code (see above)
    # False when the receiver is alive and streaming but has no position yet
    # (GGA with empty lat/lon fields — typically quality 0 / no satellites).
    # Such a fix is still reported so callers can distinguish "receiver silent"
    # from "receiver fine, still searching"; lat/lon/alt are NaN.
    position_valid: bool = True
    num_sv: int = 0
    hdop: Optional[float] = None
    # Row-major 3x3 ENU covariance + its type (mirrors NavSatFix).
    position_covariance: List[float] = field(
        default_factory=lambda: [0.0] * 9)
    position_covariance_type: int = COV_TYPE_UNKNOWN
    # ENU horizontal velocity (from SOG/COG); vertical rate is not in NMEA.
    velocity_east: float = 0.0
    velocity_north: float = 0.0
    velocity_valid: bool = False

    @property
    def has_fix(self) -> bool:
        return self.quality != GGA_QUALITY_NO_FIX

    @property
    def is_rtk(self) -> bool:
        return self.quality in (GGA_QUALITY_RTK_FIXED, GGA_QUALITY_RTK_FLOAT)


class NMEAError(RuntimeError):
    """Raised only on programmer misuse; bad sentences on the wire are skipped."""


def valid_checksum(sentence: str) -> bool:
    """Validate an NMEA ``$...*HH`` checksum (XOR of the chars between the two)."""
    star = sentence.rfind('*')
    if not sentence.startswith('$') or star < 0:
        return False
    calc = 0
    for ch in sentence[1:star]:
        calc ^= ord(ch)
    try:
        return calc == int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False


def _sentence_type(fields: List[str]) -> str:
    """Return the 3-char sentence type (e.g. 'GGA') from the address field."""
    # fields[0] is like '$GNGGA'; the type is its last three characters.
    return fields[0][-3:] if fields and len(fields[0]) >= 3 else ''


def _parse_deg(value: str, hemi: str, deg_digits: int) -> Optional[float]:
    """Convert NMEA ``d(dd)mm.mmmm`` + hemisphere to signed decimal degrees."""
    if not value:
        return None
    try:
        degrees = int(value[:deg_digits])
        minutes = float(value[deg_digits:])
    except ValueError:
        return None
    dec = degrees + minutes / 60.0
    if hemi in ('S', 'W'):
        dec = -dec
    return dec


def _to_float(value: str) -> Optional[float]:
    try:
        return float(value)
    except ValueError:
        return None


class ZedF9P:
    """Reads NMEA from a transport and yields :class:`GnssFix` on each GGA.

    GST (accuracy) and VTG/RMC (velocity) arrive as their own sentences, so we
    cache the most recent of each and merge it into the next GGA-completed fix.
    """

    def __init__(self, transport: Transport,
                 hdop_position_sigma: float = 1.0) -> None:
        self._t = transport
        # Fallback horizontal sigma (m) per unit HDOP, used only when the
        # receiver isn't emitting GST. RTK-grade GST is far better; this just
        # keeps the covariance non-degenerate for a plain fix.
        self._hdop_sigma = hdop_position_sigma
        # Cached annotations from the most recent GST / velocity sentence.
        self._std_lat: Optional[float] = None
        self._std_lon: Optional[float] = None
        self._std_alt: Optional[float] = None
        self._vel_east: Optional[float] = None
        self._vel_north: Optional[float] = None

    # -- lifecycle -----------------------------------------------------------
    def open(self) -> None:
        self._t.open()

    def close(self) -> None:
        self._t.close()

    # -- reading -------------------------------------------------------------
    def poll(self, max_lines: int = 128) -> Optional[GnssFix]:
        """Drain buffered sentences; return the newest completed fix, or None.

        Reads only what is already buffered (never blocks on an empty stream),
        up to ``max_lines`` sentences per call, so a fast node timer can keep
        latency low without letting the serial buffer back up.
        """
        newest: Optional[GnssFix] = None
        for _ in range(max_lines):
            if not self._t.has_data():
                break
            raw = self._t.readline()
            if not raw:
                break
            sentence = raw.decode('ascii', errors='ignore').strip()
            fix = self._ingest(sentence)
            if fix is not None:
                newest = fix
        return newest

    def _ingest(self, sentence: str) -> Optional[GnssFix]:
        """Parse one sentence; return a GnssFix iff it was a valid GGA."""
        if not sentence.startswith('$') or not valid_checksum(sentence):
            return None
        fields = sentence[:sentence.rfind('*')].split(',')
        kind = _sentence_type(fields)
        if kind == 'GGA':
            return self._parse_gga(fields)
        if kind == 'GST':
            self._parse_gst(fields)
        elif kind == 'VTG':
            self._parse_vtg(fields)
        elif kind == 'RMC':
            self._parse_rmc(fields)
        return None

    # -- per-sentence parsers ------------------------------------------------
    def _parse_gga(self, f: List[str]) -> Optional[GnssFix]:
        # $xxGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,geoidSep,M,age,ref
        if len(f) < 12:
            return None
        try:
            quality = int(f[6]) if f[6] else GGA_QUALITY_NO_FIX
        except ValueError:
            quality = GGA_QUALITY_NO_FIX

        num_sv = int(f[7]) if f[7].isdigit() else 0
        hdop = _to_float(f[8])

        lat = _parse_deg(f[2], f[3], 2)
        lon = _parse_deg(f[4], f[5], 3)
        if lat is None or lon is None:
            # No position yet (e.g. quality 0 with empty fields). Report it
            # rather than returning None: the satellite count is exactly the
            # signal that distinguishes a disconnected antenna (0 sats forever)
            # from a receiver that is acquiring. Covariance and velocity stay
            # unset — neither means anything without a position.
            return GnssFix(latitude=math.nan, longitude=math.nan,
                           altitude=math.nan, quality=quality,
                           position_valid=False, num_sv=num_sv, hdop=hdop)

        # NavSatFix altitude is height above the WGS-84 ellipsoid. GGA field 9
        # is orthometric height (above the geoid/MSL) and field 11 is the geoid
        # separation N, so: ellipsoidal = orthometric + N.
        msl = _to_float(f[9])
        geoid_sep = _to_float(f[11])
        altitude = 0.0
        if msl is not None:
            altitude = msl + (geoid_sep or 0.0)

        fix = GnssFix(latitude=lat, longitude=lon, altitude=altitude,
                      quality=quality, num_sv=num_sv, hdop=hdop)
        self._apply_covariance(fix)
        self._apply_velocity(fix)
        return fix

    def _parse_gst(self, f: List[str]) -> None:
        # $xxGST,time,rms,semiMajor,semiMinor,orient,stdLat,stdLon,stdAlt
        if len(f) < 9:
            return
        self._std_lat = _to_float(f[6])
        self._std_lon = _to_float(f[7])
        self._std_alt = _to_float(f[8])

    def _parse_vtg(self, f: List[str]) -> None:
        # $xxVTG,cogTrue,T,cogMag,M,sogKnots,N,sogKmh,K,mode
        if len(f) < 9:
            return
        cog = _to_float(f[1])
        speed = _to_float(f[7])                 # km/h
        speed = speed * KMH_TO_MPS if speed is not None else None
        if speed is None:
            knots = _to_float(f[5])
            speed = knots * KNOTS_TO_MPS if knots is not None else None
        self._set_velocity(cog, speed)

    def _parse_rmc(self, f: List[str]) -> None:
        # $xxRMC,time,status,lat,N/S,lon,E/W,sogKnots,cogTrue,date,...
        if len(f) < 9:
            return
        knots = _to_float(f[7])
        cog = _to_float(f[8])
        speed = knots * KNOTS_TO_MPS if knots is not None else None
        self._set_velocity(cog, speed)

    # -- annotation merge ----------------------------------------------------
    def _set_velocity(self, course_deg: Optional[float],
                      speed_mps: Optional[float]) -> None:
        if course_deg is None or speed_mps is None:
            return
        theta = math.radians(course_deg)          # course is CW from true north
        self._vel_east = speed_mps * math.sin(theta)
        self._vel_north = speed_mps * math.cos(theta)

    def _apply_velocity(self, fix: GnssFix) -> None:
        if self._vel_east is not None and self._vel_north is not None:
            fix.velocity_east = self._vel_east
            fix.velocity_north = self._vel_north
            fix.velocity_valid = True

    def _apply_covariance(self, fix: GnssFix) -> None:
        cov = [0.0] * 9  # row-major ENU: [0]=EE, [4]=NN, [8]=UU
        if (self._std_lat is not None and self._std_lon is not None
                and self._std_alt is not None):
            # GST std-devs are already in metres, per lat/lon/alt axis.
            cov[0] = self._std_lon ** 2   # East  <- longitude error
            cov[4] = self._std_lat ** 2   # North <- latitude error
            cov[8] = self._std_alt ** 2   # Up    <- altitude error
            fix.position_covariance = cov
            fix.position_covariance_type = COV_TYPE_DIAGONAL_KNOWN
        elif fix.hdop is not None:
            # No GST: approximate from HDOP. Rough, but better than "unknown"
            # for a filter — and clearly flagged as APPROXIMATED.
            h_sigma = fix.hdop * self._hdop_sigma
            cov[0] = h_sigma ** 2
            cov[4] = h_sigma ** 2
            cov[8] = (2.0 * h_sigma) ** 2   # vertical dilution ~2x horizontal
            fix.position_covariance = cov
            fix.position_covariance_type = COV_TYPE_APPROXIMATED
        else:
            fix.position_covariance = cov
            fix.position_covariance_type = COV_TYPE_UNKNOWN
