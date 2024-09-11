cmake_minimum_required(VERSION 3.8)
project(keyboard_input)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_input_msgs REQUIRED)

add_executable(keyboard_input src/KeyboardInput.cpp)
target_include_directories(keyboard_input
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
        keyboard_input PUBLIC
        rclcpp
        control_input_msgs
)

install(TARGETS
        keyboard_input
        DESTINATION lib/${PROJECT_NAME})

ament_package()
