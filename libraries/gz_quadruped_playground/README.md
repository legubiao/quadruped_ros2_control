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

## Related Materials

* [Gazebo OdometryPublisher Plugin](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html#details)