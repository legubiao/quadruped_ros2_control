# DeepRobotics Lite3 Description
This repository contains the urdf model of lite3.

![lite3](../../../.images/lite3.png)

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy

## Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to lite3_description --symlink-install
```

## Visualize the robot
To visualize and check the configuration of the robot in rviz, simply launch:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lite3_description visualize.launch.py
```

## Launch ROS2 Control
### Mujoco Simulator
* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch lite3_description unitree_guide.launch.py
  ```
* OCS2 Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch a1_description ocs2_control.launch.py
  ```
* Legged Gym Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch a1_description rl_control.launch.py
  ```


### Gazebo Simulator
* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch lite3_description gazebo_unitree_guide.launch.py
  ```
* Legged Gym Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch lite3_description gazebo_rl_control.launch.py
  ```