# Legged Gym Controller

This repository contains the reinforcement learning based controllers for the quadruped robot.

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy


## 2. Build
### 2.1 Installing libtorch
```bash
cd ~/CLionProjects/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
```
```bash
cd ~/CLionProjects/
rm -rf libtorch-shared-with-deps-latest.zip
echo 'export Torch_DIR=~/CLionProjects/libtorch' >> ~/.bashrc
```

### 2.2 Build Legged Gym Controller
```bash
cd ~/ros2_ws
colcon build --packages-up-to legged_gym_controller
```

## 3. Launch
* Unitree A1 Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch a1_description rl_control.launch.py
  ```