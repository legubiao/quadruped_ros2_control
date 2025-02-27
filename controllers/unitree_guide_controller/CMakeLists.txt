cmake_minimum_required(VERSION 3.8)
project(unitree_guide_controller)

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
        std_msgs
        control_input_msgs
        controller_common
        kdl_parser
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED

        src/UnitreeGuideController.cpp

        src/FSM/StateFixedStand.cpp
        src/FSM/StateSwingTest.cpp
        src/FSM/StateFreeStand.cpp
        src/FSM/StateBalanceTest.cpp
        src/FSM/StateTrotting.cpp

        src/robot/QuadrupedRobot.cpp
        src/robot/RobotLeg.cpp

        src/control/Estimator.cpp
        src/control/LowPassFilter.cpp
        src/control/BalanceCtrl.cpp

        src/quadProgpp/Array.cc
        src/quadProgpp/QuadProg++.cc

        src/gait/WaveGenerator.cpp
        src/gait/FeetEndCalc.cpp
        src/gait/GaitGenerator.cpp

)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${CONTROLLER_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface unitree_guide_controller.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

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
