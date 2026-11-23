# raspimouse_python_ros2

ROS 2 node for Raspberry Pi Mouse, built on [raspimouse_python_driver](https://github.com/YusukeKato/raspimouse_python_driver).

This is an unofficial package developed by volunteers.
Topic names and types follow the official [rt-net/raspimouse2](https://github.com/rt-net/raspimouse2).

## Requirements

- Raspberry Pi 4 with Ubuntu 24.04 and ROS 2 Jazzy
- The setup of [raspimouse_python_driver](https://github.com/YusukeKato/raspimouse_python_driver#setup) (config.txt, udev rule)

## Build

```bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
```

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/YusukeKato/raspimouse_python_ros2.git
vcs import < raspimouse_python_ros2/raspimouse_python_ros2.repos
```

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build
```

## Run

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch raspimouse_python_ros2 raspimouse.launch.py
```

Turn on the motor power and drive forward at 0.15 m/s:

```bash
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"
```

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.15}}}"
```

The robot stops when `cmd_vel` stops for `cmd_vel_timeout` (0.5 s).
To try the node on a PC without the robot, add `use_mock_backend:=true` to the launch command.

## Interfaces

| Name | Type | |
|---|---|---|
| `cmd_vel` | geometry_msgs/TwistStamped (Twist with `use_stamped_cmd_vel: false`) | subscribed |
| `leds` | raspimouse_msgs/Leds | subscribed |
| `buzzer` | std_msgs/Int16 (Hz, 0 stops) | subscribed |
| `motor_power` | std_srvs/SetBool | service |
| `odom` | nav_msgs/Odometry, and TF `odom` -> `base_footprint` | published |
| `light_sensors` | raspimouse_msgs/LightSensors | published |
| `switches` | raspimouse_msgs/Switches | published |
| `battery` | sensor_msgs/BatteryState (voltage only) | published |

Parameters and their defaults: [config/raspimouse.yaml](raspimouse_python_ros2/config/raspimouse.yaml).

## Test

```bash
cd ~/ros2_ws
colcon test --packages-select raspimouse_python_ros2
colcon test-result --verbose
```

## License

[MIT](LICENSE)
