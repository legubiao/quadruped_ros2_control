# RL Quadruped Controller

This repository contains the reinforcement learning based controllers for the quadruped robot.

[![](http://i0.hdslb.com/bfs/archive/9886e7f9ed06d7f880b5614cb2f4c3ec1d7bf85f.jpg)](https://www.bilibili.com/video/BV1QP1pYBE47/)

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

## 2. Build

### 2.1 Installing libtorch

> You can also choose `libtorch` with cuda. Just remember to download for c++ 11 ABI version. The position to place `libtorch` is also not fixed, just need to config the `.bashrc`.

```bash
cd ~/CLionProjects/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
```

```bash
cd ~
rm -rf libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib' >> ~/.bashrc
```

### 2.2 Build Controller

```bash
cd ~/ros2_ws
colcon build --packages-up-to rl_quadruped_controller --symlink-install
```

## 3. Launch

### 3.1 Mujoco Simulation
> **Warm Reminder**: You need to launch [Unitree Mujoco C++ Simulation](https://github.com/legubiao/unitree_mujoco) before launch the controller.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```

### 3.2 Gazebo Classic 11 (ROS2 Humble)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller gazebo_classic.launch.py pkg_description:=a1_description
```

### 3.3 Gazebo Harmonic (ROS2 Jazzy)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller gazebo.launch.py pkg_description:=go2_description
```