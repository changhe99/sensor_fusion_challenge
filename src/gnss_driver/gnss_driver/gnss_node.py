"""ROS 2 node: read the ZED-F9P NMEA stream and publish sensor_msgs/NavSatFix.

Publishes an absolute position fix on ``/gnss/fix`` (with RTK status and a
per-axis covariance derived from the receiver's GST error estimates) and,
optionally, the ENU horizontal velocity on ``/gnss/vel`` as a
geometry_msgs/TwistWithCovarianceStamped — both handy for fusing against the
BMI088 IMU stream.

Unlike the IMU node (which polls a register at a fixed rate), GNSS is
event-driven: the receiver emits sentences on its own schedule, so the timer
here just drains whatever has arrived and publishes when a new fix completes.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from gnss_driver.transport import SerialTransport
from gnss_driver.zed_f9p import (GGA_QUALITY_DGPS, GGA_QUALITY_RTK_FIXED,
                                 GGA_QUALITY_RTK_FLOAT, ZedF9P)

# ArduSimple simpleRTK2B (ZED-F9P) stable USB path — survives ttyACM re-ordering
# with the IMU board also connected. Override in config/gnss.yaml if needed.
DEFAULT_PORT = ('/dev/serial/by-id/'
                'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')


class GnssNode(Node):
    def __init__(self) -> None:
        super().__init__('gnss_node')

        # -- parameters (overridable via config/gnss.yaml) ------------------
        self.declare_parameter('frame_id', 'gnss_link')
        self.declare_parameter('fix_topic', 'gnss/fix')
        self.declare_parameter('velocity_topic', 'gnss/vel')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', 115200)   # ignored on USB CDC-ACM
        self.declare_parameter('poll_rate_hz', 20.0)  # > GNSS output rate
        self.declare_parameter('publish_velocity', True)
        # Fallback horizontal sigma (m per unit HDOP) when the receiver isn't
        # sending GST. RTK-grade GST covariance is preferred and used when seen.
        self.declare_parameter('hdop_position_sigma', 1.0)

        gp = self.get_parameter
        self._frame_id = gp('frame_id').value
        self._publish_velocity = bool(gp('publish_velocity').value)
        rate = float(gp('poll_rate_hz').value)

        transport = SerialTransport(port=gp('port').value,
                                    baudrate=int(gp('baudrate').value))
        self._gnss = ZedF9P(
            transport,
            hdop_position_sigma=float(gp('hdop_position_sigma').value))
        self._gnss.open()
        self.get_logger().info(f"ZED-F9P opened on {gp('port').value}")

        self._fix_pub = self.create_publisher(NavSatFix, gp('fix_topic').value, 10)
        self._vel_pub = None
        if self._publish_velocity:
            self._vel_pub = self.create_publisher(
                TwistWithCovarianceStamped, gp('velocity_topic').value, 10)

        # Log RTK state transitions (No fix / SPS / DGPS / RTK float / RTK fix)
        # rather than every message, so the operator sees convergence at a glance.
        self._last_quality = None

        self._timer = self.create_timer(1.0 / rate, self._on_timer)

    def _on_timer(self) -> None:
        try:
            fix = self._gnss.poll()
        except Exception as exc:  # noqa: BLE001 - log and skip this cycle
            self.get_logger().warn(f'GNSS read failed: {exc}')
            return
        if fix is None:
            return

        stamp = self.get_clock().now().to_msg()
        self._publish_fix(fix, stamp)
        if self._vel_pub is not None and fix.velocity_valid:
            self._publish_velocity_msg(fix, stamp)
        self._log_quality(fix)

    def _publish_fix(self, fix, stamp) -> None:
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        msg.status.status = self._nav_status(fix.quality)
        # Multi-band, multi-constellation receiver: GPS | GLONASS | Galileo | BeiDou.
        msg.status.service = (NavSatStatus.SERVICE_GPS
                              | NavSatStatus.SERVICE_GLONASS
                              | NavSatStatus.SERVICE_GALILEO
                              | NavSatStatus.SERVICE_COMPASS)

        msg.latitude = fix.latitude
        msg.longitude = fix.longitude
        msg.altitude = fix.altitude
        msg.position_covariance = fix.position_covariance
        msg.position_covariance_type = fix.position_covariance_type
        self._fix_pub.publish(msg)

    def _publish_velocity_msg(self, fix, stamp) -> None:
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        # ENU velocity from SOG/COG — expressed in a local ENU/earth frame,
        # not the sensor body frame. NMEA gives no vertical rate, so z = 0.
        msg.header.frame_id = self._frame_id
        msg.twist.twist.linear.x = fix.velocity_east
        msg.twist.twist.linear.y = fix.velocity_north
        msg.twist.twist.linear.z = 0.0
        self._vel_pub.publish(msg)

    @staticmethod
    def _nav_status(quality: int) -> int:
        if quality == 0:
            return NavSatStatus.STATUS_NO_FIX
        if quality in (GGA_QUALITY_DGPS, GGA_QUALITY_RTK_FIXED,
                       GGA_QUALITY_RTK_FLOAT):
            return NavSatStatus.STATUS_GBAS_FIX  # ground-based augmentation
        return NavSatStatus.STATUS_FIX

    def _log_quality(self, fix) -> None:
        if fix.quality != self._last_quality:
            names = {0: 'NO FIX', 1: 'SPS (autonomous)', 2: 'DGPS',
                     4: 'RTK FIXED', 5: 'RTK FLOAT', 6: 'dead reckoning'}
            self.get_logger().info(
                f"fix quality -> {names.get(fix.quality, fix.quality)} "
                f"({fix.num_sv} sats)")
            self._last_quality = fix.quality

    def destroy_node(self) -> bool:
        try:
            self._gnss.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
