cmake_minimum_required(VERSION 3.8)
project(ocs2_quadruped_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        rcpputils
        controller_interface
        realtime_tools

        ocs2_legged_robot_ros
        ocs2_self_collision
        control_input_msgs
        nav_msgs
        qpOASES
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/Ocs2QuadrupedController.cpp

        #        src/estimator/FromTopicEstimate.cpp
        src/estimator/LinearKalmanFilter.cpp
        src/estimator/StateEstimateBase.cpp

        src/wbc/HierarchicalWbc.cpp
        src/wbc/HoQp.cpp
        src/wbc/WbcBase.cpp
        src/wbc/WeightedWbc.cpp

        src/interface/constraint/EndEffectorLinearConstraint.cpp
        src/interface/constraint/FrictionConeConstraint.cpp
        src/interface/constraint/ZeroForceConstraint.cpp
        src/interface/constraint/NormalVelocityConstraintCppAd.cpp
        src/interface/constraint/ZeroVelocityConstraintCppAd.cpp
        src/interface/constraint/SwingTrajectoryPlanner.cpp
        src/interface/initialization/LeggedRobotInitializer.cpp
        src/interface/SwitchedModelReferenceManager.cpp
        src/interface/LeggedRobotPreComputation.cpp
        src/interface/LeggedInterface.cpp

)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${qpOASES_INCLUDE_DIR}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${CONTROLLER_INCLUDE_DEPENDS}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
