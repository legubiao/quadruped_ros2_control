# Xiaomi Cyberdog Description

This package contains the URDF description of the Xiaomi Cyberdog robot from [Cyberdog Simulator](https://github.com/MiRoboticsLab/cyberdog_simulator.git).

## Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to cyberdog_description
```

## Visualize the robot
To visualize and check the configuration of the robot in rviz, simply launch:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cyberdog_description visualize.launch.py
```

## Launch the robot in gazebo
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cyberdog_description gazebo.launch.py
```

## Launch ROS2 Control
* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch cyberdog_description unitree_guide.launch.py
  ```
* OCS2 Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch cyberdog_description ocs2_control.launch.py
  ```
