# ros2_sparkfun_otos_driver

[![Humble](https://github.com/moritzboeker/ros2_sparkfun_otos_driver/actions/workflows/humble.yml/badge.svg?branch=main)](https://github.com/moritzboeker/ros2_sparkfun_otos_driver/actions/workflows/humble.yml)
[![Jazzy](https://github.com/moritzboeker/ros2_sparkfun_otos_driver/actions/workflows/jazzy.yml/badge.svg?branch=main)](https://github.com/moritzboeker/ros2_sparkfun_otos_driver/actions/workflows/jazzy.yml)
[![Rolling](https://github.com/moritzboeker/ros2_sparkfun_otos_driver/actions/workflows/rolling.yml/badge.svg?branch=main)](https://github.com/moritzboeker/ros2_sparkfun_otos_driver/actions/workflows/rolling.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

ROS 2 driver for the [SparkFun Optical Tracking Odometry Sensor (OTOS)](https://www.sparkfun.com/products/24904),
based on the PAA5160E1 optical tracking chip and an LSM6DSO IMU.
It reads the sensor over I2C using SparkFun's
[qwiic_otos_py](https://github.com/sparkfun/qwiic_otos_py) library and publishes
[`nav_msgs/Odometry`](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html),
optionally broadcasting the `odom -> base_link` transform.

Supported distros: **Humble**, **Jazzy**, **Rolling** (pure-Python, tested in CI).

## Features

- Publishes planar pose and twist with configurable covariance on `odom`
- The twist's velocities are correctly transformed into the robot's body frame and not the sensor's tracking frame
- Broadcasts the odometry TF (can be disabled for sensor-fusion setups)
- Sensor mounting offset compensated in the sensor firmware (`offset.*` parameters)
- Per-robot linear/angular calibration scalars as parameters
- Runtime services to re-zero tracking and recalibrate the IMU
- Survives a missing or flaky sensor: retries at startup, recovers from I2C
  errors mid-run and continues from the last known pose

## Installation

```bash
# 1. Install the SparkFun sensor library (not available via rosdep)
pip install sparkfun-qwiic-otos

# 2. Clone into your workspace and build
cd ~/ros2_ws/src
git clone https://github.com/moritzboeker/ros2_sparkfun_otos_driver.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select sparkfun_otos_driver
```

The user running the node needs access to the I2C bus (typically membership in
the `i2c` group).

## Usage

```bash
ros2 launch sparkfun_otos_driver otos_launch.py
# or with your own parameter file:
ros2 launch sparkfun_otos_driver otos_launch.py params_file:=/path/to/params.yaml
```

## Interface

### Published topics

| Topic  | Type                | Description                          |
|--------|---------------------|--------------------------------------|
| `odom` | `nav_msgs/Odometry` | Planar pose and twist with covariance |

If `publish_tf` is true (default), the `frame_id -> child_frame_id` transform
is broadcast on `/tf` with every message.

### Services

| Service                | Type               | Description                                        |
|------------------------|--------------------|----------------------------------------------------|
| `~/reset_tracking`     | `std_srvs/Trigger` | Re-zero the odometry to the origin                 |
| `~/calibrate_imu`      | `std_srvs/Trigger` | Recalibrate the IMU (robot must be stationary)     |

### Parameters

| Parameter        | Type     | Default              | Description |
|------------------|----------|----------------------|-------------|
| `frame_id`       | string   | `odom`               | Odometry (parent) frame |
| `child_frame_id` | string   | `base_link`          | Robot base (child) frame |
| `publish_rate`   | double   | `20.0`               | Publish frequency in Hz |
| `publish_tf`     | bool     | `true`               | Broadcast the odometry TF. Set to `false` when another node (e.g. `robot_localization`) owns `odom -> base_link` |
| `linear_scalar`  | double   | `1.0`                | Linear calibration factor, range [0.872, 1.127] |
| `angular_scalar` | double   | `1.0`                | Angular calibration factor, range [0.872, 1.127] |
| `offset.x`       | double   | `0.0`                | Sensor x position relative to robot center [m] |
| `offset.y`       | double   | `0.0`                | Sensor y position relative to robot center [m] |
| `offset.yaw`     | double   | `0.0`                | Sensor yaw relative to robot forward direction [rad] |
| `pose_variance`  | double[] | `[1e-3, 1e-3, 1e-3]` | Initial diagonal pose covariance `[x, y, yaw]` |
| `pose_drift_per_meter` | double[] | `[5e-3, 5e-3, 5e-3]` | Pose std-dev growth per meter traveled `[x, y, yaw]` |
| `twist_variance` | double[] | `[1e-3, 1e-3, 1e-3]` | Diagonal twist covariance `[x, y, yaw]` |

See [`config/otos_params.yaml`](sparkfun_otos_driver/config/otos_params.yaml)
for a commented template.

## Calibration

The sensor's reported distances depend on its mounting height above the floor
and on part tolerances, so it may consistently over- or under-report motion by
a few percent. The scalar parameters correct this systematic scale error with
a single multiplicative factor, applied inside the sensor firmware:

```
scalar = true value / measured value
```

where *true* is what the robot actually did (measured externally) and
*measured* is what the sensor reported. For best accuracy, follow SparkFun's
[calibration procedure](https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/introduction/):

1. **Angular scalar** (calibrate this first — heading error contaminates the
   position estimate): rotate the robot in place a known number of full turns,
   e.g. 10 turns = 3600°, verified against a mark on the floor. If the
   sensor's accumulated heading reports 3564°, set
   `angular_scalar = 3600 / 3564 ≈ 1.010`.
2. **Linear scalar**: drive a known straight distance, e.g. exactly 1.000 m
   measured with a tape measure. If the sensor reports 1.020 m, set
   `linear_scalar = 1.000 / 1.020 ≈ 0.980`.
3. **Mounting offset**: measure the sensor position relative to the robot
   center and set the `offset.*` parameters; the sensor compensates for the
   offset in firmware.

Average several runs for each scalar. Valid values are limited to
[0.872, 1.127] (±12.7 %) by the firmware — if your correction falls outside
this range, check the mounting height against the datasheet instead.

The IMU is calibrated automatically at startup — keep the robot still for the
first second after launching.

## Covariance note

The OTOS reports its own standard-deviation registers, but SparkFun documents
them as "statistical quantities that do not represent actual error". This
driver therefore publishes user-tunable covariance values suited for
consumption by EKFs such as `robot_localization`.

Because odometry drift accumulates, the published pose covariance grows with
distance traveled: `var = pose_variance + (pose_drift_per_meter * distance)²`.
The drift default of `5e-3` matches SparkFun's typical accuracy spec of 0.5 %
of distance traveled; calling `~/reset_tracking` resets the accumulated
distance along with the pose. Twist covariance is static (`twist_variance`),
since velocity error does not accumulate.

## Testing

The unit tests are hardware-independent: they exercise the pure helper
functions, so no OTOS needs to be connected.

```bash
# just the unit tests, straight from source
cd ~/ros2_ws/src/ros2_sparkfun_otos_driver/sparkfun_otos_driver
python3 -m pytest test/test_otos_node.py -q

# everything CI runs: unit tests plus flake8, pep257 and copyright checks
cd ~/ros2_ws
colcon test --packages-select sparkfun_otos_driver
colcon test-result --verbose
```

Both need ROS 2 sourced and `qwiic_otos` importable. If pytest fails while
collecting with a `ModuleNotFoundError`, point `PYTHONPATH` at whatever
provides them — the package directory itself, and wherever
`sparkfun-qwiic-otos` is installed:

```bash
export PYTHONPATH="$PWD:/path/to/qwiic_otos_py:$PYTHONPATH"
```

If you work inside a Python virtualenv, deactivate it first. colcon and the
ROS message generators use the system interpreter that ROS 2 was built
against, and an active venv shadows it. The usual symptom is CMake reporting a
missing NumPy or `empy` that apt insists is already installed.

## License

MIT — see [LICENSE](LICENSE). This project uses the SparkFun `qwiic_otos_py`
library (MIT), see [THIRD_PARTY_NOTICES.txt](THIRD_PARTY_NOTICES.txt).
