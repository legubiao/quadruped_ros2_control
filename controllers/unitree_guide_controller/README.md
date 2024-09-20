# Unitree Guide Controller

This is a ros2-control controller based on unitree guide. The original unitree guide project could be
found [here](https://github.com/unitreerobotics/unitree_guide). I used KDL for the kinematic and dynamic calculation, so
the controller performance has difference with the original one (sometimes very unstable).

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble


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

## 3. Run
* [Go2 in mujoco simulation](../../descriptions/go2_description)
* [Go1/A1 in gazebo simulation](../../descriptions/quadruped_gazebo)