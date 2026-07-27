# gnss_driver

ROS 2 driver for the **u-blox ZED-F9P** (ArduSimple simpleRTK2B) RTK GNSS
receiver. Reads the receiver's NMEA stream and publishes
`sensor_msgs/msg/NavSatFix` on `/gnss/fix`, plus an optional
`geometry_msgs/msg/TwistWithCovarianceStamped` ENU velocity on `/gnss/vel`.

Like the sibling `bmi088_driver`, the protocol logic is isolated from the wiring
by a small `Transport` interface, so the same node works whether the receiver is
reached over USB serial (default), a raw UART, or a TCP/NTRIP bridge — only one
class changes.

```
gnss_driver/
  transport.py   # Transport interface + SerialTransport (Path A, USB CDC-ACM)
  zed_f9p.py     # NMEA parser + ZedF9P driver, returns fixes in deg/m/(m/s)
  gnss_node.py   # ROS 2 node: publishes NavSatFix (+ optional velocity Twist)
config/gnss.yaml # port, topics, frame_id, poll rate, covariance fallback
launch/gnss.launch.py
test/test_zed_f9p.py
```

## What it publishes

- **`/gnss/fix`** (`sensor_msgs/NavSatFix`) — latitude/longitude (WGS-84,
  degrees), altitude (metres **above the WGS-84 ellipsoid** = GGA orthometric
  height + geoid separation), an ENU diagonal covariance, and RTK status:
  - `status.status`: `STATUS_NO_FIX` / `STATUS_FIX` (SPS) / `STATUS_GBAS_FIX`
    (DGPS or RTK float/fixed).
  - `position_covariance`: from the receiver's **GST** error estimates when
    available (`DIAGONAL_KNOWN`); otherwise approximated from HDOP
    (`APPROXIMATED`), or `UNKNOWN`. The node logs each fix-quality transition
    (e.g. `RTK FLOAT -> RTK FIXED`) so you can watch RTK converge.
- **`/gnss/vel`** (`geometry_msgs/TwistWithCovarianceStamped`, optional) —
  horizontal velocity in a local **ENU** frame, derived from NMEA speed/course
  over ground (`linear.x` = East, `linear.y` = North). NMEA carries no vertical
  rate, so `linear.z` = 0.

## Transport paths

- **Path A (default, implemented): USB CDC-ACM serial.** The simpleRTK2B's USB
  enumerates as `/dev/ttyACMx`; `SerialTransport` reads it with pyserial.
- **Path B: raw UART** (ZED-F9P UART1/UART2 pins) — same `SerialTransport`, just
  point `port` at the UART and set a real `baudrate` (default 38400 on the F9P).
- **Path C: TCP / NTRIP bridge** — implement a new `Transport` subclass (three
  methods: `has_data`, `readline`, plus `open`/`close`). The NMEA parsing and
  the ROS node are unchanged.

## Path A setup (USB)

1. Connect the simpleRTK2B's **POWER+GPS** USB port to the PC. It streams NMEA
   at power-up — no receiver reconfiguration needed for a basic fix. (For
   cm-level RTK you still need corrections: an RTCM base/NTRIP feed into the
   receiver, configured with u-center — out of scope for this driver, which only
   *reads* the solution.)

2. **Find the stable device path.** With the BMI088 board also plugged in, raw
   `ttyACM` numbering isn't guaranteed (here the IMU is `ttyACM0` and the GNSS
   is `ttyACM1`), so the driver defaults to the by-id symlink:
   ```bash
   ls -l /dev/serial/by-id/     # look for ...u-blox_GNSS_receiver-if00
   ```
   Override `port` in `config/gnss.yaml` if yours differs.

3. **Verify raw sentences before ROS.** From the host or container:
   ```bash
   python3 -c "import serial; s=serial.Serial('/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',115200,timeout=1);
   [print(s.readline().decode(errors='ignore').strip()) for _ in range(10)]"
   ```
   You should see `$GNGGA,...`, `$GNRMC,...`, etc. This separates a wiring/port
   problem from a ROS problem. (`$GNGST` — needed for `DIAGONAL_KNOWN`
   covariance — is off by default on some firmware; enable it in u-center if you
   want receiver-reported accuracy instead of the HDOP fallback.)

## Build & run (Docker: Ubuntu 22.04 + ROS 2 Humble)

Same container as the IMU driver (see repo root `docker/Dockerfile` /
`docker-compose.yml`); the image bundles `pyserial`.

```bash
cd ~/git/sensor_fusion_challenge
docker compose build                 # first time / after Dockerfile changes
docker compose run --rm ros          # interactive shell in /ws (repo mounted)
```

Inside the container:

```bash
colcon build --packages-select gnss_driver
source install/setup.bash            # (also auto-sourced in new shells)
ros2 launch gnss_driver gnss.launch.py
```

The container runs `privileged` with the host `/dev` and `/dev/serial` exposed,
so pyserial reaches the receiver over USB. Inspect:

```bash
ros2 topic hz /gnss/fix          # ~= the receiver's NMEA output rate (1-10 Hz)
ros2 topic echo /gnss/fix        # check latitude/longitude/altitude + status
ros2 topic echo /gnss/vel        # ENU horizontal velocity
```

Configuration (port, baud, topics, frame_id, poll rate, covariance fallback)
lives in `config/gnss.yaml`.

## Offline tests (no hardware, no ROS)

Checksum validation, coordinate/altitude conversion, GST→covariance, RTK
quality decoding, and SOG/COG→ENU velocity are covered by a fake transport, so
they run on any Python (incl. the repo's `.venv`):

```bash
cd src/gnss_driver
python -m pytest test/test_zed_f9p.py -q
```
