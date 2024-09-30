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
- [x] OCS2 Legged Control

Video for Unitree Guide Controller:
[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)



## 1. Build
* rosdep
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```