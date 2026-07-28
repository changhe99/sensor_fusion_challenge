# ekf_fusion

Custom error-state Extended Kalman Filter (ESKF) fusing the **BMI088 IMU**
(`bmi088_driver`, `sensor_msgs/Imu` on `/imu/data_raw`) with the **ZED-F9P
GNSS** (`gnss_driver`, `sensor_msgs/NavSatFix` on `/gnss/fix` +
`geometry_msgs/TwistWithCovarianceStamped` on `/gnss/vel`). Publishes a fused
pose/velocity estimate as `nav_msgs/Odometry` on `/odometry/filtered`.

```
ekf_fusion/
  geodetic.py   # lat/lon/alt -> local ENU tangent-plane conversion
  eskf.py       # the filter itself: predict / update_position / update_velocity_horizontal
  ekf_node.py   # ROS 2 node: subscribes IMU+GNSS, publishes nav_msgs/Odometry
config/ekf_fusion.yaml
launch/ekf_fusion.launch.py
launch/fusion_demo.launch.py   # convenience: bmi088 + gnss + ekf_fusion in one launch
test/test_geodetic.py
test/test_eskf.py
```

This is a from-scratch filter (no `robot_localization`), following Joan
Solà's ["Quaternion kinematics for the error-state Kalman
filter"](https://arxiv.org/abs/1711.02508) formulation, matching this repo's
top-level goal of a *custom* state estimation implementation.

## The filter

**Nominal state** (16): position `p` (ENU, m), velocity `v` (ENU, m/s),
orientation quaternion `q` (body→ENU), accelerometer bias `b_a`, gyro bias
`b_g`.

**Error state** (15): `[δp, δv, δθ, δb_a, δb_g]` — the quantity the Kalman
covariance actually tracks. After every correction, the error estimate is
injected into the nominal state (`q ← q ⊗ exp(δθ)`, etc.) and implicitly
reset to zero.

- **Predict**, on every `/imu/data_raw` message (~200 Hz): strapdown
  integration of `p, v, q` from bias-corrected accel/gyro, plus linearized
  covariance propagation (`F`, `Q` built from the IMU noise-density params).
- **GNSS position update**: `/gnss/fix` is converted to local ENU (see
  `geodetic.py`) and used as a direct position measurement, including an
  optional IMU→antenna lever-arm offset (`gnss_lever_arm_body`, default zero
  — measure the real baseline if the antenna isn't co-located with the IMU).
- **GNSS velocity update**: `/gnss/vel`'s east/north components correct
  velocity directly. Note `gnss_driver` doesn't populate this message's
  covariance, so the node uses a fixed `gnss_velocity_variance` param instead
  of trusting the message.
- **Initialization**: the filter stays uninitialized until the first accepted
  GNSS fix (`status.status != STATUS_NO_FIX`). That fix becomes the local ENU
  origin (`p0 = 0`); roll/pitch are leveled from the most recent accel sample
  (coarse alignment via gravity direction); yaw is seeded from the GNSS
  course-over-ground if already moving faster than
  `initial_yaw_from_velocity_min_speed`, else 0 with a large initial yaw
  variance.

Implementation deliberately keeps a few things simple rather than
maximally precise — see the module docstring in `eskf.py` for exactly what
and why (block-diagonal process noise discretization, first-order rotation
Jacobian, identity reset Jacobian).

## Known limitations

- **Yaw observability.** Neither the position nor the velocity measurement
  models depend on `δθ` directly (`H` has zero columns there), so heading
  corrections only happen indirectly, through the covariance coupling built
  up by the process model — this is the standard behavior of GNSS/INS
  without a magnetometer. Yaw converges once the vehicle accelerates or
  turns; expect it to wander during a slow, straight start.
- **Yaw seeding assumes the IMU's body +x axis is the vehicle's forward
  direction.** If the BMI088 shuttle board is mounted at a different heading
  offset than the vehicle chassis, either correct for it in `bmi088_node`'s
  frame or expect the coarse yaw seed to be off by that amount (it will still
  converge once moving, per the point above).
- **Flat-Earth ENU approximation** (`geodetic.py`): accurate to well under a
  centimetre of curvature error for trajectories within a few km of the
  origin; not a general-purpose geodesy library.
- **No lever arm by default** (`gnss_lever_arm_body: [0,0,0]`) — set it once
  the physical IMU-to-antenna offset is measured.

## Build & run (Docker: Ubuntu 22.04 + ROS 2 Humble)

```bash
cd ~/git/sensor_fusion_challenge
docker compose run --rm ros            # interactive shell in /ws
colcon build --packages-select ekf_fusion
source install/setup.bash
ros2 launch ekf_fusion fusion_demo.launch.py   # imu + gnss + fusion together
# or, if the drivers are already running elsewhere:
ros2 launch ekf_fusion ekf_fusion.launch.py
```

Inspect:

```bash
ros2 topic echo /odometry/filtered
ros2 topic hz /odometry/filtered     # should match output_rate_hz
```

Configuration (topics, noise densities, GNSS aiding variances, lever arm,
initialization thresholds) lives in `config/ekf_fusion.yaml`.

## Offline tests (no hardware, no ROS)

The geodetic conversion and the filter math (strapdown mechanization,
covariance propagation, GNSS corrections) are covered by synthetic-input unit
tests, independent of ROS or message parsing:

```bash
cd src/ekf_fusion
python -m pytest test/ -q
```
