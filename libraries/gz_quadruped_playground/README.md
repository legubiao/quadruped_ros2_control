# Gazebo Quadruped Playground

This folder contains the gazebo worlds and sensor models used for quadruped robot simulation.

Tested environment:

* Ubuntu 24.04 【ROS2 Jazzy】

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to gz_quadruped_playground --symlink-install
```

## Launch Simulation
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch gz_quadruped_playground gazebo.launch.py
```