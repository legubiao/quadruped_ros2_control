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
- [x] [Learning-based Controller](controllers/rl_quadruped_controller/)
- [ ] Fully understand the RL Workflow

Video for Unitree Guide Controller:
[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)

Video for OCS2 Quadruped Controller:
[![](http://i0.hdslb.com/bfs/archive/e758ce019587032449a153cf897a543443b64bba.jpg)](https://www.bilibili.com/video/BV1UcxieuEmH/)


## 1. Quick Start
* rosdep
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
* Compile the package
    ```bash
    colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input hardware_unitree_mujoco
    ```

### 1.1 Mujoco Simulator
* Launch the unitree mujoco go2 simulation
* Launch the ros2-control
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch unitree_guide_controller mujoco.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

### 1.2 Gazebo Classic Simulator (ROS2 Humble)
* Compile Leg PD Controller
    ```bash
    colcon build --packages-up-to leg_pd_controller
    ```
* Launch the ros2-control
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch unitree_guide_controller gazebo.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

### 1.3 Gazebo Harmonic Simulator (ROS2 Jazzy)
* Compile Leg PD Controller
    ```bash
    colcon build --packages-up-to leg_pd_controller
    ```
* Launch the ros2-control
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch unitree_guide_controller gazebo_classic.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

For more details, please refer to the [unitree guide controller](controllers/unitree_guide_controller/) and [go2 description](descriptions/unitree/go2_description/).

## Reference

### Conference Paper
[1] Liao, Qiayuan, et al. "Walking in narrow spaces: Safety-critical locomotion control for quadrupedal robots with duality-based optimization." In *2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2723-2730. IEEE, 2023.

### Miscellaneous
[1] Unitree Robotics. *unitree\_guide: An open source project for controlling the quadruped robot of Unitree Robotics, and it is also the software project accompanying 《四足机器人控制算法--建模、控制与实践》 published by Unitree Robotics*. [Online]. Available: [https://github.com/unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)

[2] Qiayuan Liao. *legged\_control: An open-source NMPC, WBC, state estimation, and sim2real framework for legged robots*. [Online]. Available: [https://github.com/qiayuanl/legged_control](https://github.com/qiayuanl/legged_control)

[3] Ziqi Fan. *rl\_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.* 2024. Available: [https://github.com/fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 