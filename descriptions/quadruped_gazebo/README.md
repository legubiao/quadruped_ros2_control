# Quadruped Gazebo

This package contains the description of the quadruped robot from Unitree Robotics, and the launch file to load them into the gazebo empty world.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
  * Gazebo Harmonic

## Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to quadruped_gazebo
```

## Visualize the robot
To visualize and check the configuration of the robot in rviz, simply launch:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch quadruped_gazebo visualize.launch.py
```
The default model is a1. To use another robot model, add the following argument:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch quadruped_gazebo visualize.launch.py robot_type:=go1
```

## Launch the robot in gazebo
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch quadruped_gazebo gazebo.launch.py
```
To use another robot model, add the following argument:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch quadruped_gazebo gazebo.launch.py robot_type:=go1
```