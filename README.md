# Sensor Fusion Challenge

Custom state estimation implementation using IMU and GNSS.

## Hardware

PC: Ubuntu 24.04 (Docker with Ubuntu 22 with ROS2 Humble)

IMU: BMI088 Shuttle Board 3.0

GNSS: Ardusimple ZED-F9P

## Drivers

Each sensor has a ROS 2 (Humble) driver package under `src/`, both running in
the same Ubuntu 22.04 container (`docker/Dockerfile`, `docker-compose.yml`):

- [`bmi088_driver`](src/bmi088_driver/README.md) — publishes `sensor_msgs/Imu`
  on `/imu/data_raw` (Bosch Application Board over USB / COINES).
- [`gnss_driver`](src/gnss_driver/README.md) — publishes `sensor_msgs/NavSatFix`
  on `/gnss/fix` (+ optional ENU velocity on `/gnss/vel`) from the ZED-F9P NMEA
  stream over USB serial.

Both boards are CDC-ACM USB devices; enumeration order isn't guaranteed, so each
driver defaults to a stable `/dev/serial/by-id/` path.
