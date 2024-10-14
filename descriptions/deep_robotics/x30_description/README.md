# DeepRobotics X30 Description
This repository contains the urdf model of x30.

![x30](../../../.images/x30.png)

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy

## Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to x30_description --symlink-install
```

## Visualize the robot
To visualize and check the configuration of the robot in rviz, simply launch:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch x30_description visualize.launch.py
```

## Launch ROS2 Control

### Gazebo Simulator
* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch x30_description gazebo_unitree_guide.launch.py
  ```
* Legged Gym Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch x30_description gazebo_rl_control.launch.py
  ```