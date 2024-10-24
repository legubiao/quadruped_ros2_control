# Unitree A1 Description

This repository contains the urdf model of A1.

![A1](../../../.images/a1.png)

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to a1_description --symlink-install
```

## Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch a1_description visualize.launch.py
```

## Launch ROS2 Control

### Mujoco Simulator

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller mujoco.launch.py pkg_description:=a1_description
  ```
* OCS2 Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=a1_description
  ```
* RL Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch rl_quadruped_controller mujoco.launch.py pkg_description:=a1_description
  ```

### Gazebo Classic 11 (ROS2 Humble)

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=a1_description height:=0.43
  ```
  
### Gazebo Harmonic (ROS2 Jazzy)

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller gazebo.launch.py pkg_description:=a1_description height:=0.43
  ```
* RL Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch rl_quadruped_controller gazebo.launch.py pkg_description:=a1_description height:=0.43
  ```