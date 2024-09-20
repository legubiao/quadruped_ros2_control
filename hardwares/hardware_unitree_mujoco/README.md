# Hardware Unitree Mujoco

This package contains the hardware interface based on [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) to control the Unitree robot in Mujoco simulator. 

In theory, it also can communicate with real robot, but it is not tested yet. You can use go2 simulation in [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco).

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_unitree_mujoco
```