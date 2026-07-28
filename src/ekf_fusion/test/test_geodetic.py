"""Offline checks for the lat/lon/alt -> ENU tangent-plane conversion."""

import pytest

from ekf_fusion.geodetic import GeodeticOrigin


def test_origin_maps_to_zero():
    origin = GeodeticOrigin()
    origin.set_origin(37.4275, -122.1697, 30.0)
    e, n, u = origin.geodetic_to_enu(37.4275, -122.1697, 30.0)
    assert (e, n, u) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_one_degree_latitude_is_about_111km_north():
    origin = GeodeticOrigin()
    origin.set_origin(0.0, 0.0, 0.0)
    e, n, u = origin.geodetic_to_enu(1.0, 0.0, 0.0)
    assert e == pytest.approx(0.0, abs=1e-6)
    assert n == pytest.approx(110_574.0, rel=1e-2)
    assert u == pytest.approx(0.0, abs=1e-9)


def test_one_degree_longitude_at_equator_is_about_111km_east():
    origin = GeodeticOrigin()
    origin.set_origin(0.0, 0.0, 0.0)
    e, n, u = origin.geodetic_to_enu(0.0, 1.0, 0.0)
    assert e == pytest.approx(111_320.0, rel=1e-2)
    assert n == pytest.approx(0.0, abs=1e-6)


def test_altitude_is_pass_through_offset():
    origin = GeodeticOrigin()
    origin.set_origin(10.0, 20.0, 100.0)
    _, _, u = origin.geodetic_to_enu(10.0, 20.0, 105.5)
    assert u == pytest.approx(5.5, abs=1e-9)


def test_raises_before_origin_is_set():
    origin = GeodeticOrigin()
    with pytest.raises(RuntimeError):
        origin.geodetic_to_enu(0.0, 0.0, 0.0)
