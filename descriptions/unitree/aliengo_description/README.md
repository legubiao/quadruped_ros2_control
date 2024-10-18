# Unitree AlienGo Description

This repository contains the urdf model of Aliengo.

![Aliengo](../../../.images/aliengo.png)

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to aliengo_description  --symlink-install
```

## Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch aliengo_description visualize.launch.py
```

## Launch ROS2 Control

### Mujoco Simulator

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller mujoco.launch.py pkg_description:=aliengo_description
  ```
* OCS2 Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=aliengo_description
  ```
  
### Gazebo Simulator

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch aliengo_description gazebo.launch.py
  ```