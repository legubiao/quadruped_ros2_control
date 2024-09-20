# Leg PD Controller

This package contains a simple PD controller for the leg joints of the quadruped robot. By using this controller, other ros2-control based on position control can also work on the hardware interface which only contain effort control (for example, gazebo ros2 control).

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy

## 1. Interfaces

Provided interfaces:
* joint position
* joint velocity
* joint effort
* KP
* KD

Required hardware interfaces:
* command:
  * joint effort
* state:
  * joint position
  * joint velocity

## 2. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to leg_pd_controller
```

## 3. Run
* [Go1/A1 in gazebo simulation](../../descriptions/quadruped_gazebo)