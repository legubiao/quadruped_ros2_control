# Quadruped ROS2 Control

This repository contains the ros2-control based controllers for the quadruped robot. 
* [Controllers](controllers): contains the ros2-control controllers 
* [Commands](commands): contains command node used to send command to the controller
* [Descriptions](descriptions): contains the urdf model of the robot
* [Hardwares](hardwares): contains the ros2-control hardware interface for the robot

Todo List:
- [x] [Mujoco Simulation](hardwares/hardware_unitree_mujoco)
- [x] [Unitree Guide Controller](controllers/unitree_guide_controller)
- [x] [Gazebo Simulation](descriptions/quadruped_gazebo)
- [x] [Leg PD Controller](controllers/leg_pd_controller)
- [x] [Contact Sensor Simulation](https://github.com/legubiao/unitree_mujoco)
- [x] [OCS2 Quadruped Control](controllers/ocs2_quadruped_controller)
- [ ] Learning-based Controller

Video for Unitree Guide Controller:
[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)

Video for OCS2 Quadruped Controller:
[![](http://i0.hdslb.com/bfs/archive/e758ce019587032449a153cf897a543443b64bba.jpg)](https://www.bilibili.com/video/BV1UcxieuEmH/)


## Quick Start
* rosdep
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
* Compile the package
```bash
colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input hardware_unitree_mujoco
```
* Launch the unitree mujoco go2 simulation
* Launch the ros2-control
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_description unitree_guide.launch.py
```
* Run the keyboard control node
```bash
source ~/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```
For more details, please refer to the [unitree guide controller](controllers/unitree_guide_controller/) and [go2 description](descriptions/unitree/go2_description/).