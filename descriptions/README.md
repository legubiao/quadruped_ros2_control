# Robot Descriptions

This folder contains the URDF and SRDF files for the quadruped robot. 

* Unitree
  * [Go1](unitree/go1_description/)
  * [Go2](unitree/go2_description/)
  * [A1](unitree/a1_description/)
  * [Aliengo](unitree/aliengo_description/)
  * [B2](unitree/b2_description/)
* Xiaomi
  * [Cyberdog](xiaomi/cyberdog_description/)


## Steps to transfer urdf to Mujoco model
* Install [Mujoco](https://github.com/google-deepmind/mujoco)
* Transfer the mesh files to mujoco supported format, like stl.
* Adjust the urdf tile to match the mesh file. Transfer the mesh file from .dae to .stl may change the scale size of the mesh file.
* use `xacro` to generate the urdf file.
  ```
  xacro robot.xacro > ../urdf/robot.urdf
  ```
* use mujoco to convert the urdf file to mujoco model.
  ```
  compile robot.urdf robot.xml
  ```