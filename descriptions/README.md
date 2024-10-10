# Robot Descriptions

This folder contains the URDF and SRDF files for the quadruped robot.

* Unitree
    * [Go1](unitree/go1_description/)
    * [Go2](unitree/go2_description/)
    * [A1](unitree/a1_description/)
    * [Aliengo](unitree/aliengo_description/)
    * [B2](unitree/b2_description/)
* Xiaomi
    * [Cyberdog](xiaomi/cyberdog_description/)
* Deep Robotics
    * [Lite 3](deep_robotics/lite3_description/)

## Steps to transfer urdf to Mujoco model

* Install [Mujoco](https://github.com/google-deepmind/mujoco)
* Transfer the mesh files to mujoco supported format, like stl.
* Adjust the urdf tile to match the mesh file. Transfer the mesh file from .dae to .stl may change the scale size of the
  mesh file.
* use `xacro` to generate the urdf file.
  ```
  xacro robot.xacro > ../urdf/robot.urdf
  ```
* use mujoco to convert the urdf file to mujoco model.
  ```
  compile robot.urdf robot.xml
  ```

## Dependencies for Gazebo Simulation

Gazebo Simulation only tested on ROS2 Jazzy. I didn't add support for ROS2 Humble because the package name is different.

* Gazebo Harmonic
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```
* Ros2-Control for Gazebo
  ```bash
  sudo apt-get install ros-jazzy-gz-ros2-control
  ``` 
* Legged PD Controller
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to leg_pd_controller
    ```