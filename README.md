# FP-AbdulKarimRamadhan

ROS 2 workspace for autonomous lane following on a differential-drive robot. Uses a front camera for lane detection, a PD controller for steering, and UDP for communication with the robot hardware.

## How It Works

The workspace has a few ROS 2 nodes:

- `vision_node` handles lane detection. It converts the camera image to a bird's-eye view (BEV), filters white lane markings using HSV, then finds the left and right lane boundaries with a sliding window search. The resulting near and far lane errors are published to `vision/lane`.

- `master_node` reads the lane errors and calculates PWM and steering using a PD controller with curvature feed-forward (`kp`, `kd`, `kc`). It also reads encoder feedback to detect when the robot gets stuck. If needed, it runs a reverse recovery maneuver. Drive commands are published to `cmd_drive` at 20 Hz.

- `sender` sends drive commands over UDP to `127.0.0.1:12151`. Packets use a custom binary format with an `IRIS` header.

- `receiver` listens on port `12152` and publishes encoder and velocity feedback to `feedback`.

There's also a `keyboard_teleop` node for manual control using WASD.

The simulation is included as a Git submodule in `src/simulation`: [fira_simulation_ros2_docker](https://github.com/Internship-IRIS-ITS/fira_simulation_ros2_docker).

## Packages

| Package         | Executables                             | Description                          |
| --------------- | --------------------------------------- | ------------------------------------ |
| `vision`        | `vision_node`                           | BEV lane detection                   |
| `master`        | `master_node`                           | PD controller and recovery logic     |
| `communication` | `sender`, `receiver`, `keyboard_teleop` | UDP communication and manual control |

## Requirements

- ROS 2
- OpenCV
- `cv_bridge`, `sensor_msgs`, `std_msgs`, `rclcpp`
- Linux (POSIX sockets and terminal I/O are used)

## Setup

Clone the repository and initialize the simulation submodule:

```bash
git clone <repo-url> FP-AbdulKarimRamadhan
cd FP-AbdulKarimRamadhan
git submodule update --init --recursive
```

Build the workspace:

```bash
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
```

## Running

Start each node in a separate terminal.

**Autonomous mode**

```bash
# Terminal 1
ros2 run vision vision_node

# Terminal 2
ros2 run master master_node

# Terminal 3
ros2 launch communication communication.launch.xml
```

**Manual control**

```bash
ros2 run communication keyboard_teleop
```

Controls:

- `W` / `S`: Increase or decrease PWM
- `A` / `D`: Adjust steering
- `Space`: Stop
- `Q`: Quit

## Configuration

Nodes use ROS 2 parameters. Values can be overridden with `--ros-args -p name:=value` or a parameter file.

### master_node

| Parameter            | Default | Description                        |
| -------------------- | ------: | ---------------------------------- |
| `kp`                 |  `60.0` | Proportional gain                  |
| `kd`                 |   `8.0` | Derivative gain                    |
| `kc`                 |  `60.0` | Curvature feed-forward gain        |
| `w_near`             |   `0.3` | Near-error weight (0–1)            |
| `pwm_max`            |   `100` | Maximum forward PWM                |
| `enabled`            |  `true` | Enable controller                  |
| `lost_limit`         |   `0.6` | Time before lost-lane recovery (s) |
| `stuck_time`         |   `1.5` | Time before stuck detection (s)    |
| `stuck_speed`        |   `3.0` | Stuck speed threshold (cm/s)       |
| `reverse_pwm`        |    `40` | PWM during reverse                 |
| `reverse_min`        |   `1.0` | Minimum reverse duration (s)       |
| `reverse_max`        |   `3.0` | Maximum reverse duration (s)       |
| `reverse_steer_gain` |  `-1.0` | Steering multiplier during reverse |

### vision_node

| Parameter |  Default | Description                                                     |
| --------- | -------: | --------------------------------------------------------------- |
| `v_min`   |    `200` | Minimum HSV value for white detection                           |
| `s_max`   |     `40` | Maximum HSV saturation                                          |
| `src`     | See code | Four normalized (x, y) points for the BEV perspective transform |

### keyboard_teleop

| Parameter    | Default | Description                   |
| ------------ | ------: | ----------------------------- |
| `pwm_step`   |    `10` | PWM change per key press      |
| `steer_step` |     `5` | Steering change per key press |
| `pwm_max`    |   `255` | Maximum PWM                   |

## ROS 2 Topics

| Topic                                      | Type                         | Description                                       |
| ------------------------------------------ | ---------------------------- | ------------------------------------------------- |
| `/catvehicle/camera_front/image_raw_front` | `sensor_msgs/Image`          | Camera input                                      |
| `vision/lane`                              | `std_msgs/Float32MultiArray` | `[near_error, far_error, valid]`                  |
| `vision/bev`                               | `sensor_msgs/Image`          | BEV debug image                                   |
| `cmd_drive`                                | `std_msgs/Int32MultiArray`   | `[pwm, steer]`                                    |
| `feedback`                                 | `std_msgs/Int32MultiArray`   | `[left_ticks, right_ticks, vel_x_cms, vel_y_cms]` |

## UDP Protocol

The communication nodes use UDP with a fixed 4-byte `IRIS` header. Integer fields are little-endian `int32`.

| Packet   |    Port |     Size | Contents                                                    |
| -------- | ------: | -------: | ----------------------------------------------------------- |
| Command  | `12151` | 12 bytes | `IRIS` + PWM + steering                                     |
| Feedback | `12152` | 20 bytes | `IRIS` + left ticks + right ticks + X velocity + Y velocity |

## Project Structure

```text
src/
├── vision/           # Lane detection
├── master/           # Controller
├── communication/    # UDP and keyboard control
│   ├── include/communication/udp_header.hpp
│   └── launch/communication.launch.xml
└── simulation/       # Simulation submodule
```

## Utility Scripts

| Script                         | Description                                                       |
| ------------------------------ | ----------------------------------------------------------------- |
| `bootstrap.sh`                 | Creates the three ROS 2 packages using `ros2 pkg create`          |
| `update_simulation.sh`         | Resets and updates the simulation submodule                       |
| `update_template_workspace.sh` | Rebases the current branch onto `Internship-IRIS-ITS/iris_fp1_ws` |
