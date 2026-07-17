# bmi088_driver

ROS 2 driver for the **Bosch BMI088** (Shuttle Board 3.0) IMU. Reads the
accelerometer and gyroscope and publishes `sensor_msgs/msg/Imu` on
`/imu/data_raw`.

The sensor logic is isolated from the wiring by a small `Transport` interface,
so the same node works whether the shuttle board is connected through a Bosch
Application Board (default), a Raspberry Pi, or a USB-SPI/I2C adapter — only one
class changes.

```
bmi088_driver/
  transport.py     # Transport interface + CoinesTransport (Path A, USB)
  bmi088.py        # register-level BMI088 driver, returns SI units
  bmi088_node.py   # ROS 2 node: reads + publishes sensor_msgs/Imu
config/bmi088.yaml # ranges, ODRs, frame_id, topic, rate, covariances
launch/bmi088.launch.py
test/test_bmi088.py
```

> ⚠️ **The BMI088 is 1.8 V / 3.3 V only — it is NOT 5 V tolerant.** Never wire
> the shuttle board to 5 V logic without level shifting.

## Hardware paths

The Shuttle Board 3.0 has no USB and no MCU — it only breaks out the BMI088
pins. Pick how it reaches the PC:

- **Path A (default, implemented): Bosch Application Board 3.0/3.1 over USB.**
  Uses the COINES SDK via `coinespy` (`CoinesTransport`).
- **Path B: Raspberry Pi / Jetson** wired to the shuttle SPI/I2C.
- **Path C: PC + USB-SPI/I2C adapter** (FT232H + `pyftdi`, or MCP2221).

For B/C, implement a new `Transport` subclass (three methods: `open`, `read`,
`write`) and swap it in `bmi088_node.py`. The BMI088 register logic and the ROS
node are unchanged. I2C addresses: **accel `0x18`** (alt `0x19`), **gyro `0x68`**
(alt `0x69`).

## Path A setup (Application Board + COINES)

1. Plug the BMI088 Shuttle Board 3.0 into the Application Board socket (align the
   keyed corner; press down with both thumbs). Connect USB.

2. **Docker USB access (the important part).** coinespy identifies the board
   through libudev, so the container needs all of: `privileged`, `/dev/bus/usb`,
   the `/dev/ttyACM0` device, **and** `/run/udev` (read-only). All four are set
   in `docker-compose.yml`. Symptom if `/run/udev` is missing: every coinespy
   call returns `COINES_SUCCESS` but `get_board_info()` is all zeros and reads
   fail with `COINES_E_UNABLE_OPEN_DEVICE` (looks like a dead sensor, but isn't).

   Confirm the board is seen (non-zero `ShuttleID`, `102` = BMI088 shuttle):
   ```bash
   python3 -c "import coinespy as cpy; b=cpy.CoinesBoard(); \
   b.open_comm_interface(cpy.CommInterface.USB); print('ShuttleID', b.get_board_info().ShuttleID)"
   ```

   The board must be running `coines_bridge` firmware (the APP3.1 default;
   enumerates as USB `108c:ab38`). It only needs (re)flashing if `ShuttleID` is
   zero even on the **bare-metal host** — the firmware + flasher ship in the pip
   package under `firmware/app3.1/coines_bridge/` (see `update_coines_bridge_flash_fw.sh`).

3. `coinespy` is already installed in the container image.

4. **Verify raw reads before ROS.** From `/ws`:
   ```bash
   python3 -c "import sys;sys.path.insert(0,'src/bmi088_driver');\
   from bmi088_driver.bmi088 import BMI088; from bmi088_driver.transport import CoinesTransport;\
   imu=BMI088(CoinesTransport()); imu.open(); print(imu.read_accel(), imu.read_gyro()); imu.close()"
   ```
   Confirm accel magnitude ≈ 9.8 m/s² at rest and gyro ≈ 0 when still before
   launching the node — it separates wiring/firmware issues from ROS issues.

## Build & run (Docker: Ubuntu 22.04 + ROS 2 Humble)

The host is Ubuntu 24.04, but ROS 2 Humble targets Ubuntu 22.04 / Python 3.10,
so everything runs in a container (see `docker/Dockerfile` and
`docker-compose.yml` at the repo root). The image already bundles `coinespy`
and `imu_filter_madgwick`.

```bash
cd ~/git/sensor_fusion_challenge
docker compose build                 # first time only
docker compose run --rm ros          # interactive shell in /ws (repo mounted)
```

Then, inside the container:

```bash
colcon build --packages-select bmi088_driver
source install/setup.bash            # (also auto-sourced in new shells)
ros2 launch bmi088_driver bmi088.launch.py
```

The container runs `privileged` with `/dev/bus/usb` mounted, so `coinespy`
reaches the Application Board over USB. Build artifacts (`build/ install/ log/`)
land in the mounted repo and persist between runs.

Inspect:

```bash
ros2 topic hz /imu/data_raw      # steady rate ~= publish_rate_hz
ros2 topic echo /imu/data_raw    # accel |a| ~= 9.8 at rest; gyro ~= 0 when still
```

Configuration (ranges, ODRs, rate, frame_id, covariances) lives in
`config/bmi088.yaml`. Orientation is intentionally left unset
(`orientation_covariance[0] = -1`): the BMI088 has no magnetometer. To get a
fused orientation, feed `/imu/data_raw` into `imu_filter_madgwick`
(already installed in the container as `ros-humble-imu-filter-madgwick`).

## Offline tests (no hardware, no ROS)

The register byte-assembly, chip-ID checks, and SI conversions are covered by a
fake transport, so they run on any Python (incl. the 3.14 venv):

```bash
cd src/bmi088_driver
python -m pytest test/test_bmi088.py -q
```
