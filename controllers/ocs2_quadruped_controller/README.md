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
* qpOASES
  ```bash
  cd ~/ros2_ws/src/quadruped_ros2
  git submodule update --init --recursive
  ```
  also add below line to the Cmakelist.txt of qpOASES
  ```cmake
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  ```

### 2.2 Build OCS2 Quadruped Controller

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadruped_controller
```

## 3. Launch

* Go2 Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch go2_description ocs2_control.launch.py
  ```
