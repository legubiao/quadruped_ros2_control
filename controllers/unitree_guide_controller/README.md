# Unitree Guide Controller

This is a ros2-control controller based on unitree guide. The original unitree guide project could be
found [here](https://github.com/unitreerobotics/unitree_guide). I used KDL for the kinematic and dynamic calculation, so
the controller performance has difference with the original one (sometimes very unstable).

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)

## 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
    * joint velocity
    * joint effort
    * KP
    * KD
* state:
    * joint effort
    * joint position
    * joint velocity
    * imu sensor
      * linear acceleration
      * angular velocity
      * orientation

## 2. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_guide_controller
```