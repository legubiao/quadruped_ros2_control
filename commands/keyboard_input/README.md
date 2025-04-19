# Keyboard Input

This node will read the keyboard input and publish a control_input_msgs/Input message.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

### Build Command
```bash
cd ~/ros2_ws
colcon build --packages-up-to keyboard_input
```

### Launch Command
```bash
source ~/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

## 1. Use Instructions for Unitree Guide
### 1.1 Control Mode
* Passive Mode: Keyboard 1
* Fixed Stand: Keyboard 2
    * Free Stand: Keyboard 3
    * Trot: Keyboard 4
    * SwingTest: Keyboard 5
    * Balance: Keyboard 6
### 1.2 Control Input
* WASD IJKL: Move robot
* Space: Reset Speed Input