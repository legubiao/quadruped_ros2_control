# Basic Quadruped Controller

This is a ros2-control controller based on unitree guide. The original unitree guide project could be
found [here](https://github.com/unitreerobotics/unitree_guide). This controller has been refactored to provide improved architecture and parameter management compared to the original unitree_guide_controller.

[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)

## 1. Controller Architecture

### 1.1 Overview
The `basic_quadruped_controller` is the main controller that replaces the previous `unitree_guide_controller`. It features:

- **Unified Parameter Management**: All gain parameters (stand, swing, stable) are managed as arrays for better organization
- **Configurable Gait Height**: Gait height can be configured via ROS parameters
- **Improved FSM Structure**: Better state machine organization with cleaner parameter passing
- **Enhanced Maintainability**: More modular and maintainable code structure

### 1.2 State Machine
The controller implements a finite state machine (FSM) with the following states:
- **Passive**: Initial state, no control
- **FixedDown**: Robot lying down with fixed position
- **FixedStand**: Robot standing with fixed position
- **FreeStand**: Robot standing with balance control
- **Trotting**: Dynamic trotting gait
- **BalanceTest**: Balance testing mode

### 1.3 Key Parameters
All parameters can be configured via ROS parameters or launch files:

```yaml
# Gain parameters (kp, kd)
stand_gains: [80.0, 3.5]      # Stand state gains
swing_gains: [3.0, 2.0]       # Swing phase gains  
stable_gains: [0.8, 0.8]      # Stable phase gains

# Gait parameters
gait_height: 0.08             # Step height for trotting gait

# Robot positions
stand_pos: [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]  # Standing joint positions
down_pos: [0.0, 1.3, -2.4, 0.0, 1.3, -2.4, 0.0, 1.3, -2.4, 0.0, 1.3, -2.4]        # Lying down joint positions
```

### 1.4 Parameter Management
The controller uses a unified approach for managing gain parameters:
- **stand_gains_[0]**: Position gain (kp) for stand states
- **stand_gains_[1]**: Velocity gain (kd) for stand states
- **swing_gains_[0]**: Position gain (kp) for swing phase
- **swing_gains_[1]**: Velocity gain (kd) for swing phase
- **stable_gains_[0]**: Position gain (kp) for stable phase
- **stable_gains_[1]**: Velocity gain (kd) for stable phase

## 2. Interfaces

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

## 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to basic_quadruped_controller --symlink-install
```

## 4. Launch

### 4.1 Mujoco Simulation
> **Warm Reminder**: You need to launch [Unitree Mujoco C++ Simulation](https://github.com/legubiao/unitree_mujoco) before launch the controller.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```

### 4.2 Gazebo Harmonic
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_quadruped_controller gazebo.launch.py pkg_description:=go2_description
```

## 5. Configuration

### 5.1 Launch File Parameters
You can customize the controller behavior by modifying the launch file parameters:

```python
# Example launch file parameters
stand_gains = [80.0, 3.5]      # [kp, kd] for stand states
swing_gains = [3.0, 2.0]       # [kp, kd] for swing phase
stable_gains = [0.8, 0.8]      # [kp, kd] for stable phase
gait_height = 0.08             # Step height for trotting
```

### 5.2 Runtime Parameter Tuning
Parameters can also be adjusted at runtime using ROS2 parameter tools:

```bash
# View current parameters
ros2 param list /basic_quadruped_controller

# Set parameters
ros2 param set /basic_quadruped_controller stand_gains "[100.0, 5.0]"
ros2 param set /basic_quadruped_controller gait_height 0.1
```