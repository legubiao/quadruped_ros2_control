import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_description = "go2_description"

def process_xacro():
    pkg_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'GAZEBO': 'true'})
    return robot_description_config.toxml()


def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory(package_description), "config", "visualize_urdf.rviz")

    robot_description = process_xacro()

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'robot', '-allow_renaming', 'true', '-z', '0.4'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pd_controller",
                   "--controller-manager", "/controller_manager"],
    )

    controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_quadruped_controller", "--controller-manager", "/controller_manager"],
        parameters=[
            {
                'config_folder': os.path.join(get_package_share_directory(package_description), 'config',
                                              'legged_gym'),
            }],
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        robot_state_publisher,
        gz_spawn_entity,
        leg_pd_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[controller, imu_sensor_broadcaster, joint_state_publisher],
            )
        ),
    ])
