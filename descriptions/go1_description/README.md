# Unitree Go1 Description
This repository contains the urdf model of go1.

## Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to go1_description
```

## Visualize the robot
To visualize and check the configuration of the robot in rviz, simply launch:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go1_description visualize.launch.py
```

## Launch Hardware Interface
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go1_description hardware.launch.py
```
