# Quadruped ROS2 Control

This repository contains the ros2-control based controllers for the quadruped robot.

* [Controllers](controllers): contains the ros2-control controllers
* [Commands](commands): contains command node used to send command to the controller
* [Hardwares](hardwares): contains the ros2-control hardware interface for the robot

> **Note:** This repository only contains `go2_description` as a sample robot model to keep the package size minimal for deployment on real robots. For more robot configurations and models, please refer to [fiveages-sim/robot_descriptions](https://github.com/fiveages-sim/robot_descriptions).

> **Warning:** Default branch was developed under ROS2 Jazzy. For ROS2 Humble, please check out **humble** branch.

> **Note:** The `unitree_guide_controller` has been replaced by `basic_quadruped_controller` which provides the same functionality with improved architecture and parameter management.

Todo List:

- [x] **[2025-02-23]** Add Gazebo Playground
  - [x] OCS2 controller for Gazebo Simulation
  - [x] Refactor FSM and Basic Quadruped Controller
- [x] **[2025-03-30]** Add Real Go2 Robot Support
- [x] **[2025-05-20]** Isaac Sim Support
- [ ] OCS2 Perceptive locomotion demo

Video on Real Unitree Go2 Robot:
[![](http://i0.hdslb.com/bfs/archive/7d3856b3c5e5040f24990d3eab760cf8ba4cf80d.jpg)](https://www.bilibili.com/video/BV1QpZaY8EYV/)

## 1. Quick Start

### 1.1 Prerequisites

#### Install Pinocchio
Pinocchio is required for the basic quadruped controller. 

* [Pinocchio Official Page](https://stack-of-tasks.github.io/pinocchio/download.html)

> **Warm Reminder**: Please follow the guide in the link to install Pinocchio before building the project

#### Install other dependencies
* rosdep
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

### 1.2 Compile the package
```bash
colcon build --packages-up-to basic_quadruped_controller go2_description keyboard_input --symlink-install
```

### 1.3 Mujoco Simulator or Real Unitree Robot
> **Warning:** CycloneDDS ROS2 RMW may conflict with unitree_sdk2. If you cannot launch unitree mujoco simulation
> without `sudo`, then you cannot used `unitree_mujoco_hardware`. This conflict could be solved by one of below two
> methods:
> 1. Uninstall CycloneDDS ROS2 RMW, used another ROS2 RMW, such as FastDDS **[Recommended]**.
> 2. Follow the guide in [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) to configure the ROS2 RMW by
     compiling cyclone dds.

* Compile Unitree Hardware Interfaces
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to hardware_unitree_sdk2
    ```
* Follow the guide in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco) to launch the unitree mujoco go2
  simulation
* Launch the ros2-control
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch basic_quadruped_controller mujoco.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

![mujoco](.images/mujoco.png)

### 1.4 Gazebo Harmonic Simulator

* Install Gazebo
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```

* Compile Gazebo Playground
  ```bash
  colcon build --packages-up-to gz_quadruped_playground --symlink-install
  ```
* Launch the ros2-control
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch basic_quadruped_controller gazebo.launch.py
  ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```
  https://github.com/user-attachments/assets/a24a8309-38a9-451b-86b7-51d19d892ffc



For more details, please refer to the [basic quadruped controller](controllers/basic_quadruped_controller/)
and [go2 description](go2_description/).

## What's Next
Congratulations! You have successfully launched the quadruped robot in the simulation. Here are some suggestions for you to have a try:
* **More Robot Models** could be found at [description](descriptions/)
* **Try more controllers**. 
  * [OCS2 Quadruped Controller](controllers/ocs2_quadruped_controller): Robust MPC-based controller for quadruped robot
  * [RL Quadruped Controller](controllers/rl_quadruped_controller): Reinforcement learning controller for quadruped robot
* **Simulate with more sensors**
  * [Gazebo Quadruped Playground](libraries/gz_quadruped_playground): Provide gazebo simulation with lidar or depth camera.
* **Real Robot Deploy**
  * [Go2 Robot](go2_description): Check here about how to deploy on go2 robot.

## Reference

### Conference Paper

[1] Liao, Qiayuan, et al. "Walking in narrow spaces: Safety-critical locomotion control for quadrupedal robots with
duality-based optimization." In *2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp.
2723-2730. IEEE, 2023.

### Miscellaneous

[1] Unitree Robotics. *unitree\_guide: An open source project for controlling the quadruped robot of Unitree Robotics,
and it is also the software project accompanying 《四足机器人控制算法--建模、控制与实践》 published by Unitree
Robotics*. [Online].
Available: [https://github.com/unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)

[2] Qiayuan Liao. *legged\_control: An open-source NMPC, WBC, state estimation, and sim2real framework for legged
robots*. [Online]. Available: [https://github.com/qiayuanl/legged_control](https://github.com/qiayuanl/legged_control)

[3] Ziqi Fan. *rl\_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.*
2024. Available: [https://github.com/fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 
