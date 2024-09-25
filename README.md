# Quadruped ROS2 Control

This repository contains the ros2-control based controllers for the quadruped robot. 
* [Controllers](controllers): contains the ros2-control controllers 
* [Commands](commands): contains command node used to send command to the controller
* [Descriptions](descriptions): contains the urdf model of the robot
* [Hardwares](hardwares): contains the ros2-control hardware interface for the robot

Todo List:
- [x] Mujoco Simulation
- [x] Unitree Guide Controller
- [x] Gazebo Simulation
- [x] Leg PD Controller
- [x] Contact Sensor
- [ ] OCS2 Legged Control


## 1. Build
* rosdep
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```