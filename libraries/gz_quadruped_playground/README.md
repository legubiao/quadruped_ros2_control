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

* Unitree Guide Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py
  ```
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py world:=warehouse
   ```
* OCS2 Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py controller:=ocs2
  ```
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py controller:=ocs2 world:=warehouse
   ```

## SLAM Test

### Record Rosbag

```bash
cd ~/ros2_ws
ros2 bag record /rgbd_d435/points /rgbd_d435/depth_image /scan/points /imu_sensor_broadcaster/imu /odom /tf /tf_static /joint_states
```

### Fast LIO

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch gz_quadruped_playground fast_lio.launch.py
```

## Related Materials

* [Gazebo OdometryPublisher Plugin](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html#details)
* [Gazebo Intel Realsense D435 RGBD camera](https://app.gazebosim.org/OpenRobotics/fuel/models/Intel%20RealSense%20D435)