cmake_minimum_required(VERSION 3.8)
project(quadruped_gazebo)


find_package(ament_cmake REQUIRED)

install(
        DIRECTORY meshes urdf launch config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
