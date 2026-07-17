"""ROS 2 node: read the BMI088 and publish sensor_msgs/Imu.

Publishes *raw* IMU data (linear acceleration + angular velocity) on
``/imu/data_raw``. Orientation is left unset (orientation_covariance[0] = -1):
the BMI088 has no magnetometer, so an absolute orientation is not available
here — feed this topic into e.g. imu_filter_madgwick to obtain one.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from bmi088_driver.bmi088 import BMI088
from bmi088_driver.transport import CoinesTransport


class Bmi088Node(Node):
    def __init__(self) -> None:
        super().__init__('bmi088_node')

        # -- parameters (overridable via config/bmi088.yaml) ----------------
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', 'imu/data_raw')
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('acc_range_g', 6)
        self.declare_parameter('acc_odr_hz', 200)
        self.declare_parameter('gyro_range_dps', 2000)
        self.declare_parameter('gyro_odr_hz', 200)
        self.declare_parameter('vdd', 3.3)
        self.declare_parameter('vddio', 3.3)
        # Diagonal covariances; set <0 to mark "unknown". Tune from a static log.
        self.declare_parameter('linear_acceleration_variance', 0.01)
        self.declare_parameter('angular_velocity_variance', 0.001)

        gp = self.get_parameter
        self._frame_id = gp('frame_id').value
        rate = float(gp('publish_rate_hz').value)
        accel_var = float(gp('linear_acceleration_variance').value)
        gyro_var = float(gp('angular_velocity_variance').value)

        transport = CoinesTransport(vdd=float(gp('vdd').value),
                                    vddio=float(gp('vddio').value))
        self._imu = BMI088(
            transport,
            acc_range_g=int(gp('acc_range_g').value),
            acc_odr_hz=int(gp('acc_odr_hz').value),
            gyro_range_dps=int(gp('gyro_range_dps').value),
            gyro_odr_hz=int(gp('gyro_odr_hz').value),
        )
        self._imu.open()
        self.get_logger().info('BMI088 initialized (accel + gyro online)')

        self._pub = self.create_publisher(Imu, gp('topic').value, 50)

        # Pre-build the covariance arrays once.
        self._accel_cov = [accel_var, 0.0, 0.0,
                           0.0, accel_var, 0.0,
                           0.0, 0.0, accel_var]
        self._gyro_cov = [gyro_var, 0.0, 0.0,
                          0.0, gyro_var, 0.0,
                          0.0, 0.0, gyro_var]

        self._timer = self.create_timer(1.0 / rate, self._on_timer)

    def _on_timer(self) -> None:
        try:
            ax, ay, az = self._imu.read_accel()
            gx, gy, gz = self._imu.read_gyro()
        except Exception as exc:  # noqa: BLE001 - log and skip a sample
            self.get_logger().warn(f'BMI088 read failed: {exc}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = self._accel_cov

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = self._gyro_cov

        # No magnetometer -> orientation unknown.
        msg.orientation_covariance[0] = -1.0

        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._imu.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bmi088Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
