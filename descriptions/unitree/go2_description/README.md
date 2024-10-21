# Unitree Go2 Description

This repository contains the urdf model of go2.

![go2](../../../.images/go2.png)

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to go2_description --symlink-install
```

## Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_description visualize.launch.py
```

## Launch ROS2 Control

### Mujoco Simulator

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller mujoco.launch.py
  ```
* OCS2 Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_quadruped_controller mujoco.launch.py
  ```
* RL Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch rl_quadruped_controller mujoco.launch.py
  ```

### Gazebo Classic 11 (ROS2 Humble)
* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller gazebo_classic.launch.py
  ```

### Gazebo Harmonic (ROS2 Jazzy)

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller gazebo.launch.py
  ```
* RL Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch go2_description gazebo_rl_control.launch.py
  ```

## When used for isaac gym or other similiar engine

Collision parameters in urdf can be amended to better train the robot:

Open "go2_description.urdf" in "./go2_description/urdf",
and amend the ` box size="0.213 0.0245 0.034" ` in links of "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh".

For example, change previous values to ` box size="0.11 0.0245 0.034" ` means the length of the thigh is shortened from
0.213 to 0.11, which can avoid unnecessary collision between the thigh link and the calf link.

The collision model before and after the above amendment are shown as "Normal_collision_model.png" and "
Amended_collision_model.png" respectively.


