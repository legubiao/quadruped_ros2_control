# Gazebo Quadruped ROS2 Control Plugin

This repository is a modified version of [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)

## Build

* [Install Gazebo Harmonic at Ubuntu 22.04](https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu)
* install ros gz
  ```bash
  sudo apt-get install ros-humble-ros-gzharmonic
  ```
* Build ROS2 Control plugin
  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to gz_quadruped_hardware --symlink-install
  ```