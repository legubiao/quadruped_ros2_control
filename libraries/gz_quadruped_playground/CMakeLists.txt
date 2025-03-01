cmake_minimum_required(VERSION 3.8)
project(gz_quadruped_playground)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY worlds launch config models
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()