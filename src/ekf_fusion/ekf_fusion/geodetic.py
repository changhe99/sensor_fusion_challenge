"""Geodetic (lat/lon/alt) -> local ENU conversion, anchored at a fixed origin.

Uses a flat-Earth / local-tangent-plane approximation about the origin
latitude (WGS-84 meridian + prime-vertical radii of curvature), not a full
ECEF round-trip. That keeps trajectories within roughly a few km of the
origin accurate to well under a centimetre of curvature error — more than
enough for a vehicle test area — while avoiding a geodesy library dependency.
Deliberately ROS-free (like ``bmi088.py`` / ``zed_f9p.py``) so it can be
unit-tested without ROS or hardware.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

WGS84_A = 6378137.0                       # semi-major axis, metres
WGS84_F = 1.0 / 298.257223563              # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)       # eccentricity squared


class GeodeticOrigin:
    """A local ENU frame anchored at one lat/lon/alt origin."""

    def __init__(self) -> None:
        self._lat0_rad: Optional[float] = None
        self._lon0_rad: Optional[float] = None
        self._alt0: float = 0.0
        self._meridian_radius = 0.0     # M: north/south (latitude) radius
        self._prime_vertical_radius = 0.0  # N: east/west (longitude) radius

    @property
    def is_set(self) -> bool:
        return self._lat0_rad is not None

    def set_origin(self, lat0_deg: float, lon0_deg: float, alt0: float) -> None:
        self._lat0_rad = math.radians(lat0_deg)
        self._lon0_rad = math.radians(lon0_deg)
        self._alt0 = alt0

        sin_lat0 = math.sin(self._lat0_rad)
        denom = math.sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0)
        self._meridian_radius = (WGS84_A * (1.0 - WGS84_E2)) / denom ** 3
        self._prime_vertical_radius = WGS84_A / denom

    def geodetic_to_enu(self, lat_deg: float, lon_deg: float,
                        alt: float) -> Tuple[float, float, float]:
        """Return (east, north, up) metres relative to the origin."""
        if not self.is_set:
            raise RuntimeError('set_origin() must be called before use')

        dlat = math.radians(lat_deg) - self._lat0_rad
        dlon = math.radians(lon_deg) - self._lon0_rad

        north = dlat * (self._meridian_radius + self._alt0)
        east = dlon * (self._prime_vertical_radius + self._alt0) * math.cos(
            self._lat0_rad)
        up = alt - self._alt0
        return east, north, up
