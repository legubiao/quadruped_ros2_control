# Joystick Input

This node will listen to the joystick topic and publish a control_input_msgs/Input message.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
  * Logitech F310 Gamepad

```bash
cd ~/ros2_ws
colcon build --packages-up-to joystick_input
```

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch joystick_input joystick.launch.py
```

## 1. Use Instructions for Unitree Guide

### 1.1 Control Mode

* Passive Mode: LB + B
* Fixed Stand: LB + A
    * Free Stand: LB + X
    * Trot: LB + Y
    * SwingTest: LT + B
    * Balance: LT + A

### 1.2 Control Input

* WASD IJKL: Move robot
* Space: Reset Speed Input