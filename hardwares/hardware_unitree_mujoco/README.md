# Hardware Unitree Mujoco

This package contains the hardware interface based on [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) to control the Unitree robot in Mujoco simulator. 

In theory, it also can communicate with real robot, but it is not tested yet. You can use go2 simulation in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco). In this simulation, I add foot force sensor support.

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
  * foot force sensor

## 2. Build

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

Build Command:
```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_unitree_mujoco
```