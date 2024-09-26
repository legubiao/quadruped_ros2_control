# Unitree AlienGo Description
This repository contains the urdf model of Aliengo.

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy

## Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to aliengo_description
```

## Visualize the robot
To visualize and check the configuration of the robot in rviz, simply launch:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch aliengo_description visualize.launch.py
```

## Launch Hardware Interface
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch aliengo_description hardware.launch.py
```
