"""Offline checks for the ZED-F9P NMEA driver (no hardware, no ROS).

A fake transport feeds canned NMEA sentences so we can verify checksum
validation, the coordinate/altitude conversions, the GST -> covariance mapping,
RTK quality decoding, and SOG/COG -> ENU velocity.
"""

import math

import pytest

from gnss_driver.transport import Transport
from gnss_driver.zed_f9p import (COV_TYPE_APPROXIMATED, COV_TYPE_DIAGONAL_KNOWN,
                                 COV_TYPE_UNKNOWN, GGA_QUALITY_RTK_FIXED,
                                 ZedF9P, valid_checksum)


def nmea(body: str) -> str:
    """Append the correct ``*HH`` checksum to a ``$...`` sentence body."""
    calc = 0
    for ch in body[1:]:
        calc ^= ord(ch)
    return f'{body}*{calc:02X}'


class FakeSerial(Transport):
    """Serves queued NMEA lines (bytes), then reports empty."""

    def __init__(self, lines):
        self._queue = [ln.encode('ascii') + b'\r\n' for ln in lines]

    def open(self):
        pass

    def close(self):
        pass

    def has_data(self):
        return bool(self._queue)

    def readline(self):
        return self._queue.pop(0) if self._queue else b''


def test_valid_checksum():
    good = nmea('$GNGGA,172814.0,3723.46587704,N,12202.26957864,W,'
                '4,10,0.9,18.0,M,-25.6,M,1.0,0000')
    assert valid_checksum(good)
    # Flip a payload char -> checksum must now fail.
    corrupted = good.replace('3723', '3724', 1)
    assert not valid_checksum(corrupted)
    assert not valid_checksum('garbage')


def test_gga_position_and_rtk_quality():
    # lat 37 deg 23.46587704', lon -122 deg 02.26957864', quality 4 (RTK fixed).
    line = nmea('$GNGGA,172814.0,3723.46587704,N,12202.26957864,W,'
                '4,10,0.9,18.0,M,-25.6,M,1.0,0000')
    gnss = ZedF9P(FakeSerial([line]))
    fix = gnss.poll()
    assert fix is not None
    assert fix.quality == GGA_QUALITY_RTK_FIXED
    assert fix.is_rtk and fix.has_fix
    assert fix.latitude == pytest.approx(37 + 23.46587704 / 60.0, abs=1e-8)
    assert fix.longitude == pytest.approx(-(122 + 2.26957864 / 60.0), abs=1e-8)
    # Ellipsoidal altitude = orthometric (18.0) + geoid separation (-25.6).
    assert fix.altitude == pytest.approx(18.0 - 25.6, abs=1e-6)
    assert fix.num_sv == 10


def test_south_west_hemisphere_signs():
    line = nmea('$GPGGA,000000.0,3350.00000000,S,15112.00000000,E,'
                '1,08,1.0,50.0,M,0.0,M,,')
    fix = ZedF9P(FakeSerial([line])).poll()
    assert fix.latitude == pytest.approx(-(33 + 50.0 / 60.0), abs=1e-8)
    assert fix.longitude == pytest.approx(151 + 12.0 / 60.0, abs=1e-8)


def test_gst_populates_diagonal_covariance():
    # GST std-devs (m): lat 0.030, lon 0.040, alt 0.050. GST must precede the
    # GGA it annotates, matching how the driver caches then merges.
    gst = nmea('$GNGST,172814.0,0.0,0.05,0.03,0.0,0.030,0.040,0.050')
    gga = nmea('$GNGGA,172814.0,3723.46587704,N,12202.26957864,W,'
               '4,10,0.9,18.0,M,-25.6,M,1.0,0000')
    fix = ZedF9P(FakeSerial([gst, gga])).poll()
    assert fix.position_covariance_type == COV_TYPE_DIAGONAL_KNOWN
    assert fix.position_covariance[0] == pytest.approx(0.040 ** 2)  # East<-lon
    assert fix.position_covariance[4] == pytest.approx(0.030 ** 2)  # North<-lat
    assert fix.position_covariance[8] == pytest.approx(0.050 ** 2)  # Up<-alt


def test_covariance_falls_back_to_hdop_without_gst():
    gga = nmea('$GNGGA,172814.0,3723.46587704,N,12202.26957864,W,'
               '1,08,1.5,18.0,M,-25.6,M,,')
    fix = ZedF9P(FakeSerial([gga]), hdop_position_sigma=2.0).poll()
    assert fix.position_covariance_type == COV_TYPE_APPROXIMATED
    assert fix.position_covariance[0] == pytest.approx((1.5 * 2.0) ** 2)


def test_vtg_velocity_to_enu():
    # Heading 090 deg (due east) at 3.6 km/h = 1.0 m/s -> vE=+1, vN=0.
    vtg = nmea('$GNVTG,90.0,T,,M,1.9438,N,3.6,K,D')
    gga = nmea('$GNGGA,172814.0,3723.46587704,N,12202.26957864,W,'
               '4,10,0.9,18.0,M,-25.6,M,1.0,0000')
    fix = ZedF9P(FakeSerial([vtg, gga])).poll()
    assert fix.velocity_valid
    assert fix.velocity_east == pytest.approx(1.0, abs=1e-3)
    assert fix.velocity_north == pytest.approx(0.0, abs=1e-3)


def test_no_fix_gga_reports_searching_not_silence():
    # Quality 0 with empty lat/lon: still reported, flagged position_valid=False
    # with NaN coordinates, so the node can publish STATUS_NO_FIX instead of
    # going silent. num_sv/HDOP are still parsed - they're the diagnostic.
    line = nmea('$GNGGA,172814.0,,,,,0,00,99.99,,M,,M,,')
    fix = ZedF9P(FakeSerial([line])).poll()
    assert fix is not None
    assert not fix.position_valid
    assert not fix.has_fix
    assert fix.num_sv == 0
    assert fix.hdop == pytest.approx(99.99)
    assert math.isnan(fix.latitude) and math.isnan(fix.longitude)
    assert math.isnan(fix.altitude)
    # No position -> covariance is meaningless and must stay UNKNOWN.
    assert fix.position_covariance_type == COV_TYPE_UNKNOWN


def test_no_fix_reports_satellites_being_acquired():
    # Receiver sees satellites but hasn't solved a position yet - the state
    # that tells you the antenna is fine and it just needs more time.
    line = nmea('$GNGGA,172814.0,,,,,0,04,5.5,,M,,M,,')
    fix = ZedF9P(FakeSerial([line])).poll()
    assert not fix.position_valid
    assert fix.num_sv == 4


def test_no_fix_does_not_reuse_stale_velocity():
    # A valid VTG followed by a position-less GGA must not attach the old
    # velocity to a fix that has no position.
    vtg = nmea('$GNVTG,90.0,T,,M,1.9438,N,3.6,K,D')
    gga = nmea('$GNGGA,172814.0,,,,,0,00,99.99,,M,,M,,')
    fix = ZedF9P(FakeSerial([vtg, gga])).poll()
    assert not fix.position_valid
    assert not fix.velocity_valid


def test_poll_returns_newest_of_multiple_fixes():
    older = nmea('$GNGGA,172814.0,3723.00000000,N,12202.00000000,W,'
                 '1,08,1.0,10.0,M,0.0,M,,')
    newer = nmea('$GNGGA,172815.0,3724.00000000,N,12203.00000000,W,'
                 '4,10,0.9,20.0,M,0.0,M,1.0,0000')
    fix = ZedF9P(FakeSerial([older, newer])).poll()
    assert fix.latitude == pytest.approx(37 + 24.0 / 60.0, abs=1e-8)
    assert fix.quality == GGA_QUALITY_RTK_FIXED
