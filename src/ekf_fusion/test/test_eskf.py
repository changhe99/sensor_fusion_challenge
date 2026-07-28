"""Offline checks for the error-state EKF (no ROS, no hardware).

Synthetic IMU inputs exercise the strapdown mechanization and the
covariance propagation/correction directly, independent of message
parsing or transport.
"""

import numpy as np
import pytest

from ekf_fusion.eskf import ESKF, level_from_accel, quat_from_euler

LEVEL_ACCEL = np.array([0.0, 0.0, 9.80665])  # stationary reading, level
ZERO_GYRO = np.zeros(3)


def _init(eskf, p0=(0.0, 0.0, 0.0), v0=(0.0, 0.0, 0.0)):
    eskf.initialize(
        p0=np.array(p0), v0=np.array(v0), q0=np.array([1.0, 0.0, 0.0, 0.0]),
        position_var=np.full(3, 1.0), velocity_var=np.full(3, 1.0),
        attitude_var=np.array([0.01, 0.01, 1.0]))


def test_stationary_state_stays_at_origin():
    eskf = ESKF()
    _init(eskf)
    for _ in range(200):
        eskf.predict(LEVEL_ACCEL, ZERO_GYRO, dt=0.005)
    assert eskf.p == pytest.approx(np.zeros(3), abs=1e-9)
    assert eskf.v == pytest.approx(np.zeros(3), abs=1e-9)


def test_constant_velocity_integrates_position():
    eskf = ESKF()
    _init(eskf, v0=(1.0, 0.0, 0.0))
    dt = 0.005
    steps = 2000  # 10 s
    for _ in range(steps):
        eskf.predict(LEVEL_ACCEL, ZERO_GYRO, dt=dt)
    assert eskf.v == pytest.approx(np.array([1.0, 0.0, 0.0]), abs=1e-9)
    assert eskf.p == pytest.approx(np.array([10.0, 0.0, 0.0]), rel=1e-6)


def test_gnss_position_update_pulls_diverged_state_toward_truth():
    eskf = ESKF()
    _init(eskf)
    # Simulate accumulated drift away from the true (0,0,0) position.
    eskf.p = np.array([5.0, 0.0, 0.0])
    eskf.P[0:3, 0:3] = np.eye(3) * 4.0  # 2 m std prior

    eskf.update_position(np.zeros(3), np.eye(3) * 0.01)  # confident GNSS fix

    assert eskf.p[0] < 2.5  # moved more than halfway back toward truth
    assert abs(eskf.p[0]) < abs(5.0)


def test_gnss_velocity_update_corrects_velocity():
    eskf = ESKF()
    _init(eskf, v0=(3.0, 3.0, 0.0))
    eskf.P[3:6, 3:6] = np.eye(3) * 4.0

    eskf.update_velocity_horizontal(np.array([0.0, 0.0]), np.eye(2) * 0.01)

    assert abs(eskf.v[0]) < 1.5
    assert abs(eskf.v[1]) < 1.5


def test_covariance_stays_symmetric_and_psd():
    eskf = ESKF()
    _init(eskf, v0=(1.0, 0.5, 0.0))
    for _ in range(500):
        eskf.predict(LEVEL_ACCEL, np.array([0.01, -0.01, 0.02]), dt=0.005)
    eskf.update_position(np.array([1.0, 0.5, 0.0]), np.eye(3) * 0.05)
    eskf.update_velocity_horizontal(np.array([1.0, 0.5]), np.eye(2) * 0.02)

    assert eskf.P == pytest.approx(eskf.P.T, abs=1e-9)
    eigvals = np.linalg.eigvalsh(eskf.P)
    assert np.all(eigvals >= -1e-8)


def test_level_from_accel_recovers_known_tilt():
    true_roll, true_pitch = 0.1, -0.2
    q = quat_from_euler(true_roll, true_pitch, 0.4)  # yaw shouldn't matter
    from ekf_fusion.eskf import quat_to_rotmat
    R = quat_to_rotmat(q)
    accel_body = R.T @ np.array([0.0, 0.0, 1.0])  # body-frame "up" direction

    roll, pitch = level_from_accel(accel_body)
    assert roll == pytest.approx(true_roll, abs=1e-6)
    assert pitch == pytest.approx(true_pitch, abs=1e-6)


def test_gyro_bias_estimate_grows_only_from_updates_not_predict():
    # With no GNSS corrections, bias states shouldn't move (only their
    # covariance grows) - predict() only integrates the nominal state.
    eskf = ESKF()
    _init(eskf)
    for _ in range(100):
        eskf.predict(LEVEL_ACCEL, np.array([0.001, 0.0, 0.0]), dt=0.005)
    assert eskf.bg == pytest.approx(np.zeros(3), abs=1e-12)
    assert eskf.ba == pytest.approx(np.zeros(3), abs=1e-12)
