# OCS2 Quadruped Controller

This is a ros2-control controller based on [legged_control](https://github.com/qiayuanl/legged_control)
and [ocs2_ros2](https://github.com/legubiao/ocs2_ros2).

## 2. Build

### 2.1 Build Dependencies

* OCS2 ROS2 Libraries
  ```bash
  colcon build --packages-up-to ocs2_legged_robot_ros
  colcon build --packages-up-to ocs2_self_collision
  ```

### 2.2 Build OCS2 Quadruped Controller

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadruped_controller
```

## 3. Launch
* Unitree Go1 Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch go1_description ocs2_control.launch.py
  ```
* Unitree Aliengo Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch aliengo_description ocs2_control.launch.py
  ```
* Unitree Go2 Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch go2_description ocs2_control.launch.py
  ```
