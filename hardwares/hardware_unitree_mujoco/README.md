# Hardware Unitree Mujoco

This package contains the hardware interface based on [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) to control the Unitree robot in Mujoco simulator. 

In theory, it also can communicate with real robot, but it is not tested yet. You can use go2 simulation in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco). In this simulation, I add foot force sensor support.

*[x] **[2025-01-16]** Add odometer states for simulation. 

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
colcon build --packages-up-to hardware_unitree_mujoco --symlink-install
```

## 3. Config network and domain
Since the real unitree robot has different network and domain name, you need to set the network and domain name in the xacro file.
```xml
<hardware>
    <plugin>hardware_unitree_mujoco/HardwareUnitree</plugin>
    <param name="domain">1</param>
    <param name="network_interface">lo</param>
</hardware>
```

After modified the config, you can tried to visualize the robot info from real robot by following command:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hardware_unitree_mujoco visualize.launch.py
```