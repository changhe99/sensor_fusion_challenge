"""ROS 2 node: fuse BMI088 IMU + ZED-F9P GNSS with a custom error-state EKF.

Subscribes ``imu/data_raw`` (sensor_msgs/Imu, drives the predict step at IMU
rate), ``gnss/fix`` (sensor_msgs/NavSatFix, position aiding + filter
initialization) and ``gnss/vel`` (geometry_msgs/TwistWithCovarianceStamped,
horizontal velocity aiding). Publishes the fused estimate as
nav_msgs/Odometry on ``odometry/filtered``.

The filter stays uninitialized until the first accepted GNSS fix: that fix
becomes the local ENU origin, position/velocity are seeded from it, roll/
pitch are leveled from the most recent accel sample, and yaw is seeded from
the GNSS course-over-ground if the vehicle is already moving (else 0 with a
large yaw variance) - see ekf_fusion/eskf.py and the package README for why
yaw is only weakly observable from GNSS position/velocity alone.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

from ekf_fusion.eskf import (ESKF, ESKFConfig, level_from_accel,
                             quat_from_euler, quat_to_rotmat)
from ekf_fusion.geodetic import GeodeticOrigin


class EkfNode(Node):
    def __init__(self) -> None:
        super().__init__('ekf_node')

        # -- parameters (overridable via config/ekf_fusion.yaml) ------------
        self.declare_parameter('imu_topic', 'imu/data_raw')
        self.declare_parameter('gnss_fix_topic', 'gnss/fix')
        self.declare_parameter('gnss_vel_topic', 'gnss/vel')
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'imu_link')
        self.declare_parameter('output_rate_hz', 50.0)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('max_predict_dt', 0.1)

        # IMU noise-density params. Placeholders from the BMI088 datasheet;
        # tune from a static log (same guidance as bmi088_driver's config).
        self.declare_parameter('accel_noise_density', 0.0019)
        self.declare_parameter('gyro_noise_density', 0.00025)
        self.declare_parameter('accel_bias_random_walk', 0.0006)
        self.declare_parameter('gyro_bias_random_walk', 0.0001)
        self.declare_parameter('accel_bias_prior_variance', 1e-2)
        self.declare_parameter('gyro_bias_prior_variance', 1e-4)

        # GNSS aiding params.
        # gnss_driver doesn't populate /gnss/vel's twist covariance, so a
        # fixed fallback variance is used for the velocity update instead of
        # trusting the (all-zero) message covariance.
        self.declare_parameter('gnss_velocity_variance', 0.05)
        self.declare_parameter('gnss_position_variance_floor', 0.0004)
        self.declare_parameter('gnss_lever_arm_body', [0.0, 0.0, 0.0])
        self.declare_parameter('gnss_vel_max_age_s', 1.0)

        # Coarse-alignment / initialization params.
        self.declare_parameter('initial_yaw_from_velocity_min_speed', 0.5)
        self.declare_parameter('initial_velocity_variance_fallback', 4.0)
        self.declare_parameter('initial_roll_pitch_std_deg', 5.0)
        self.declare_parameter('initial_yaw_std_deg_unseeded', 180.0)
        self.declare_parameter('initial_yaw_std_deg_seeded', 15.0)

        gp = self.get_parameter
        self._frame_id = gp('frame_id').value
        self._child_frame_id = gp('child_frame_id').value
        self._publish_tf = bool(gp('publish_tf').value)
        self._max_predict_dt = float(gp('max_predict_dt').value)
        self._gnss_velocity_variance = float(gp('gnss_velocity_variance').value)
        self._gnss_position_variance_floor = float(
            gp('gnss_position_variance_floor').value)
        self._lever_arm = np.array(gp('gnss_lever_arm_body').value, dtype=float)
        self._gnss_vel_max_age = float(gp('gnss_vel_max_age_s').value)
        self._yaw_min_speed = float(
            gp('initial_yaw_from_velocity_min_speed').value)
        self._initial_velocity_var_fallback = float(
            gp('initial_velocity_variance_fallback').value)
        self._roll_pitch_var = math.radians(
            float(gp('initial_roll_pitch_std_deg').value)) ** 2
        self._yaw_var_unseeded = math.radians(
            float(gp('initial_yaw_std_deg_unseeded').value)) ** 2
        self._yaw_var_seeded = math.radians(
            float(gp('initial_yaw_std_deg_seeded').value)) ** 2
        self._accel_bias_prior_var = float(
            gp('accel_bias_prior_variance').value)
        self._gyro_bias_prior_var = float(gp('gyro_bias_prior_variance').value)

        config = ESKFConfig(
            accel_noise_density=float(gp('accel_noise_density').value),
            gyro_noise_density=float(gp('gyro_noise_density').value),
            accel_bias_random_walk=float(gp('accel_bias_random_walk').value),
            gyro_bias_random_walk=float(gp('gyro_bias_random_walk').value),
        )
        self._eskf = ESKF(config)
        self._origin = GeodeticOrigin()

        self._last_imu_stamp: Time | None = None
        self._last_stamp_msg = None
        self._last_accel: np.ndarray | None = None
        self._last_gyro: np.ndarray | None = None
        self._last_gnss_vel = None  # (east, north, Time) cache for init/aging

        self.create_subscription(Imu, gp('imu_topic').value, self._on_imu, 50)
        self.create_subscription(
            NavSatFix, gp('gnss_fix_topic').value, self._on_fix, 10)
        self.create_subscription(
            TwistWithCovarianceStamped, gp('gnss_vel_topic').value,
            self._on_vel, 10)

        self._odom_pub = self.create_publisher(Odometry, gp('odom_topic').value, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        rate = float(gp('output_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._on_publish_timer)

    # -- IMU: predict --------------------------------------------------------
    def _on_imu(self, msg: Imu) -> None:
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                         msg.angular_velocity.z])
        self._last_accel = accel
        self._last_gyro = gyro
        self._last_stamp_msg = msg.header.stamp

        if not self._eskf.initialized:
            return

        stamp = Time.from_msg(msg.header.stamp)
        if self._last_imu_stamp is None:
            self._last_imu_stamp = stamp
            return

        dt = (stamp - self._last_imu_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            self.get_logger().warn(f'non-positive IMU dt ({dt:.4f}s); skipping')
            return
        if dt > self._max_predict_dt:
            self.get_logger().warn(
                f'large IMU gap ({dt:.3f}s); clamping to {self._max_predict_dt}s')
            dt = self._max_predict_dt

        self._eskf.predict(accel, gyro, dt)
        self._last_imu_stamp = stamp

    # -- GNSS velocity: aiding update -----------------------------------------
    def _on_vel(self, msg: TwistWithCovarianceStamped) -> None:
        east = msg.twist.twist.linear.x
        north = msg.twist.twist.linear.y
        self._last_gnss_vel = (east, north, Time.from_msg(msg.header.stamp))

        if self._eskf.initialized:
            R = np.eye(2) * self._gnss_velocity_variance
            self._eskf.update_velocity_horizontal(np.array([east, north]), R)

    # -- GNSS fix: initialization / position aiding ---------------------------
    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return

        if not self._origin.is_set:
            self._origin.set_origin(msg.latitude, msg.longitude, msg.altitude)
        east, north, up = self._origin.geodetic_to_enu(
            msg.latitude, msg.longitude, msg.altitude)
        enu = np.array([east, north, up])

        pos_cov = np.array(msg.position_covariance, dtype=float).reshape(3, 3).copy()
        for i in range(3):
            if pos_cov[i, i] < self._gnss_position_variance_floor:
                pos_cov[i, i] = self._gnss_position_variance_floor

        if not self._eskf.initialized:
            self._initialize_filter(enu, pos_cov, msg.header.stamp)
            return

        self._eskf.update_position(enu, pos_cov, lever_arm_body=self._lever_arm)

    def _initialize_filter(self, enu: np.ndarray, pos_cov: np.ndarray,
                           fix_stamp) -> None:
        v0 = np.zeros(3)
        yaw0 = 0.0
        yaw_var = self._yaw_var_unseeded
        vel_var = np.full(3, self._initial_velocity_var_fallback)

        if self._last_gnss_vel is not None:
            east, north, vel_stamp = self._last_gnss_vel
            age = abs((Time.from_msg(fix_stamp) - vel_stamp).nanoseconds) * 1e-9
            if age <= self._gnss_vel_max_age:
                v0[0], v0[1] = east, north
                vel_var = np.full(3, self._gnss_velocity_variance)
                speed = math.hypot(east, north)
                if speed >= self._yaw_min_speed:
                    # ENU: heading of the (east, north) velocity vector,
                    # measured the same way as quat_from_euler's yaw (CCW
                    # from the east/+x axis). Assumes the IMU body +x axis
                    # is mounted facing the vehicle's forward direction.
                    yaw0 = math.atan2(north, east)
                    yaw_var = self._yaw_var_seeded

        roll, pitch = 0.0, 0.0
        if self._last_accel is not None:
            roll, pitch = level_from_accel(self._last_accel)
        q0 = quat_from_euler(roll, pitch, yaw0)

        attitude_var = np.array([self._roll_pitch_var, self._roll_pitch_var, yaw_var])
        self._eskf.initialize(
            p0=enu, v0=v0, q0=q0,
            position_var=np.diag(pos_cov).copy(), velocity_var=vel_var,
            attitude_var=attitude_var,
            accel_bias_var=self._accel_bias_prior_var,
            gyro_bias_var=self._gyro_bias_prior_var)

        seeded = yaw_var == self._yaw_var_seeded
        self.get_logger().info(
            f'ESKF initialized at first GNSS fix; yaw0={math.degrees(yaw0):.1f} deg '
            f'({"seeded from GNSS velocity" if seeded else "unseeded, large variance"})')

    # -- publish ---------------------------------------------------------------
    def _on_publish_timer(self) -> None:
        if not self._eskf.initialized:
            return
        state = self._eskf.get_state()
        p, v_world, q, P = state['p'], state['v'], state['q'], state['P']
        R = quat_to_rotmat(q)

        odom = Odometry()
        odom.header.stamp = (self._last_stamp_msg if self._last_stamp_msg is not None
                             else self.get_clock().now().to_msg())
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id

        odom.pose.pose.position.x = float(p[0])
        odom.pose.pose.position.y = float(p[1])
        odom.pose.pose.position.z = float(p[2])
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.pose.covariance = self._pose_covariance(P)

        # Twist is reported in the child (body) frame per nav_msgs/Odometry
        # convention; the filter's nominal velocity is world/ENU, so rotate.
        v_body = R.T @ v_world
        odom.twist.twist.linear.x = float(v_body[0])
        odom.twist.twist.linear.y = float(v_body[1])
        odom.twist.twist.linear.z = float(v_body[2])
        if self._last_gyro is not None:
            gyro_unbiased = self._last_gyro - state['bg']
            odom.twist.twist.angular.x = float(gyro_unbiased[0])
            odom.twist.twist.angular.y = float(gyro_unbiased[1])
            odom.twist.twist.angular.z = float(gyro_unbiased[2])
        odom.twist.covariance = self._twist_covariance(P, R)

        self._odom_pub.publish(odom)
        if self._publish_tf:
            self._broadcast_tf(odom)

    @staticmethod
    def _pose_covariance(P: np.ndarray) -> list:
        cov = np.zeros((6, 6))
        cov[0:3, 0:3] = P[0:3, 0:3]
        cov[3:6, 3:6] = P[6:9, 6:9]  # body-frame small-angle attitude error
        return cov.flatten().tolist()

    @staticmethod
    def _twist_covariance(P: np.ndarray, R: np.ndarray) -> list:
        cov = np.zeros((6, 6))
        cov[0:3, 0:3] = R.T @ P[3:6, 3:6] @ R  # rotate velocity cov into body frame
        cov[3:6, 3:6] = P[12:15, 12:15]  # gyro-bias uncertainty as an angular-rate proxy
        return cov.flatten().tolist()

    def _broadcast_tf(self, odom: Odometry) -> None:
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
