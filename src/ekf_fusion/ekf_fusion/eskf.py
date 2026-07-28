"""Error-state EKF (ESKF) fusing strapdown IMU mechanization with GNSS aiding.

Follows Sola's "Quaternion kinematics for the error-state Kalman filter"
formulation: a 16-element *nominal* state integrated by the (nonlinear)
strapdown equations, plus a 15-element *error* state whose (linear, Gaussian)
covariance is what the Kalman machinery actually propagates/corrects. After
each correction the error estimate is injected into the nominal state and
implicitly reset to zero.

Nominal state:  p (ENU position, 3), v (ENU velocity, 3),
                q (body->ENU unit quaternion, [w, x, y, z]),
                b_a (accel bias, 3), b_g (gyro bias, 3)
Error state:    [dp, dv, dtheta, db_a, db_g]  (15,)

Deliberately ROS-free (like ``bmi088.py`` / ``zed_f9p.py``) so the filter math
can be unit-tested without ROS, hardware, or even real sensor data.

Simplifications made deliberately, for a tractable/testable implementation:
- Process noise is discretized as sigma^2 * dt per axis (block-diagonal), not
  the full Van Loan cross-coupled form.
- The state-transition's rotation sub-block uses the first-order
  (I - skew(w) * dt) approximation rather than the exact matrix exponential;
  adequate given a ~5 ms IMU period.
- The covariance "reset Jacobian" after injecting a rotation correction is
  approximated as identity (the exact form is I - 0.5*skew(dtheta), a
  second-order effect for the small corrections a well-tuned filter makes).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

STATE_DIM = 15  # error-state size: [dp(3), dv(3), dtheta(3), dba(3), dbg(3)]
GRAVITY = 9.80665


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y],
                     [z, 0.0, -x],
                     [-y, x, 0.0]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 (x) q2, both [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Rotation matrix (body -> world) for unit quaternion [w, x, y, z]."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotvec_to_quat(phi: np.ndarray) -> np.ndarray:
    """Quaternion for the rotation vector phi (exact, not small-angle)."""
    angle = np.linalg.norm(phi)
    if angle < 1e-12:
        return np.array([1.0, 0.5 * phi[0], 0.5 * phi[1], 0.5 * phi[2]])
    axis = phi / angle
    half = 0.5 * angle
    s = np.sin(half)
    return np.array([np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body->world quaternion for aerospace roll/pitch/yaw (Rz*Ry*Rx)."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def level_from_accel(accel_body: np.ndarray) -> Tuple[float, float]:
    """Coarse-alignment roll/pitch from a (near-stationary) accel reading.

    accel_body approximates R^T @ [0, 0, +1] (body-frame direction of world
    "up", since a stationary accelerometer reads the reaction to gravity).
    Yaw is not observable this way - the caller must supply it separately.
    """
    a = accel_body / np.linalg.norm(accel_body)
    roll = np.arctan2(a[1], a[2])
    pitch = np.arctan2(-a[0], np.sqrt(a[1] ** 2 + a[2] ** 2))
    return roll, pitch


@dataclass
class ESKFConfig:
    accel_noise_density: float = 0.0019       # m/s^2 / sqrt(Hz)
    gyro_noise_density: float = 0.00025       # rad/s / sqrt(Hz)
    accel_bias_random_walk: float = 0.0006    # m/s^3 / sqrt(Hz)
    gyro_bias_random_walk: float = 0.0001     # rad/s^2 / sqrt(Hz)
    gravity: float = GRAVITY


class ESKF:
    def __init__(self, config: Optional[ESKFConfig] = None) -> None:
        self._cfg = config or ESKFConfig()
        self.initialized = False

        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        self.P = np.zeros((STATE_DIM, STATE_DIM))

    # -- initialization --------------------------------------------------
    def initialize(self, p0: np.ndarray, v0: np.ndarray, q0: np.ndarray,
                   position_var: np.ndarray, velocity_var: np.ndarray,
                   attitude_var: np.ndarray,
                   accel_bias_var: float = 1e-2,
                   gyro_bias_var: float = 1e-4) -> None:
        """Seed the nominal state and diagonal initial covariance.

        position_var/velocity_var/attitude_var are each length-3 (per-axis
        variances); attitude_var is [roll_var, pitch_var, yaw_var] (rad^2).
        """
        self.p = np.array(p0, dtype=float)
        self.v = np.array(v0, dtype=float)
        self.q = quat_normalize(np.array(q0, dtype=float))
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)

        diag = np.concatenate([
            np.asarray(position_var, dtype=float),
            np.asarray(velocity_var, dtype=float),
            np.asarray(attitude_var, dtype=float),
            np.full(3, accel_bias_var),
            np.full(3, gyro_bias_var),
        ])
        self.P = np.diag(diag)
        self.initialized = True

    # -- prediction --------------------------------------------------------
    def predict(self, accel_meas: np.ndarray, gyro_meas: np.ndarray,
                dt: float) -> None:
        if not self.initialized or dt <= 0.0:
            return

        am = np.asarray(accel_meas, dtype=float) - self.ba
        wm = np.asarray(gyro_meas, dtype=float) - self.bg

        R = quat_to_rotmat(self.q)
        gravity_world = np.array([0.0, 0.0, -self._cfg.gravity])
        a_world = R @ am + gravity_world

        # -- nominal state propagation --
        self.p = self.p + self.v * dt + 0.5 * a_world * dt * dt
        self.v = self.v + a_world * dt
        self.q = quat_normalize(quat_multiply(self.q, rotvec_to_quat(wm * dt)))

        # -- error-state covariance propagation --
        Fx = np.eye(STATE_DIM)
        Fx[0:3, 3:6] = np.eye(3) * dt
        Fx[3:6, 6:9] = -R @ skew(am) * dt
        Fx[3:6, 9:12] = -R * dt
        Fx[6:9, 6:9] = np.eye(3) - skew(wm) * dt
        Fx[6:9, 12:15] = -np.eye(3) * dt

        Q = np.zeros((STATE_DIM, STATE_DIM))
        Q[3:6, 3:6] = np.eye(3) * (self._cfg.accel_noise_density ** 2) * dt
        Q[6:9, 6:9] = np.eye(3) * (self._cfg.gyro_noise_density ** 2) * dt
        Q[9:12, 9:12] = np.eye(3) * (self._cfg.accel_bias_random_walk ** 2) * dt
        Q[12:15, 12:15] = np.eye(3) * (self._cfg.gyro_bias_random_walk ** 2) * dt

        self.P = Fx @ self.P @ Fx.T + Q

    # -- corrections ---------------------------------------------------------
    def update_position(self, meas_enu: np.ndarray, meas_cov: np.ndarray,
                        lever_arm_body: Optional[np.ndarray] = None) -> None:
        if not self.initialized:
            return
        R = quat_to_rotmat(self.q)
        lever = (np.zeros(3) if lever_arm_body is None
                 else np.asarray(lever_arm_body, dtype=float))
        predicted = self.p + R @ lever

        H = np.zeros((3, STATE_DIM))
        H[:, 0:3] = np.eye(3)
        if np.any(lever):
            H[:, 6:9] = -R @ skew(lever)

        y = np.asarray(meas_enu, dtype=float) - predicted
        self._correct(y, H, np.asarray(meas_cov, dtype=float))

    def update_velocity_horizontal(self, meas_east_north: np.ndarray,
                                   meas_cov: np.ndarray) -> None:
        if not self.initialized:
            return
        H = np.zeros((2, STATE_DIM))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        y = np.asarray(meas_east_north, dtype=float) - self.v[0:2]
        self._correct(y, H, np.asarray(meas_cov, dtype=float))

    def _correct(self, y: np.ndarray, H: np.ndarray, R_meas: np.ndarray) -> None:
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y

        self.p = self.p + dx[0:3]
        self.v = self.v + dx[3:6]
        self.q = quat_normalize(quat_multiply(self.q, rotvec_to_quat(dx[6:9])))
        self.ba = self.ba + dx[9:12]
        self.bg = self.bg + dx[12:15]

        I_KH = np.eye(STATE_DIM) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T

    # -- accessors ---------------------------------------------------------
    def get_state(self) -> dict:
        return {
            'p': self.p.copy(),
            'v': self.v.copy(),
            'q': self.q.copy(),
            'ba': self.ba.copy(),
            'bg': self.bg.copy(),
            'P': self.P.copy(),
        }
