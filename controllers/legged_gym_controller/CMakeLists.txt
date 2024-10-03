cmake_minimum_required(VERSION 3.8)
project(legged_gym_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

# rl_sdk library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Torch REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_library(rl_sdk library/rl_sdk/rl_sdk.cpp)
target_include_directories(rl_sdk
        PUBLIC
        library/rl_sdk)
target_link_libraries(rl_sdk "${TORCH_LIBRARIES}" Python3::Python Python3::Module)
set_property(TARGET rl_sdk PROPERTY CXX_STANDARD 14)
find_package(Python3 COMPONENTS NumPy)
if (Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk Python3::NumPy)
else ()
    target_compile_definitions(rl_sdk WITHOUT_NUMPY)
endif ()

set(dependencies
        pluginlib
        rcpputils
        controller_interface
        realtime_tools
        control_input_msgs
)

# find dependencies
foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()


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
