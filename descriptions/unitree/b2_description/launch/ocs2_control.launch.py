import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from sympy.physics.vector.printing import params

package_description = "b2_description"
package_controller = "ocs2_quadruped_controller"

def process_xacro(context):
    robot_type_value = context.launch_configurations['robot_type']
    pkg_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'robot_type': robot_type_value})
    return (robot_description_config.toxml(), robot_type_value)


def launch_setup(context, *args, **kwargs):
    (robot_description, robot_type) = process_xacro(context)
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control.yaml",
        ]
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

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers,
                    {
                        'urdf_file': os.path.join(get_package_share_directory(package_description), 'urdf', 'robot.urdf'),
                        'task_file': os.path.join(get_package_share_directory(package_description), 'config', 'ocs2',
                                                  'task.info'),
                        'reference_file': os.path.join(get_package_share_directory(package_description), 'config',
                                                       'ocs2', 'reference.info'),
                        'gait_file': os.path.join(get_package_share_directory(package_description), 'config',
                                                       'ocs2', 'gait.info')
                    }],
        output="both",
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

    ocs2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_quadruped_controller", "--controller-manager", "/controller_manager"]
    )

    return [
        robot_state_publisher,
        controller_manager,
        joint_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[ocs2_controller],
            )
        ),
    ]


def generate_launch_description():
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='b2',
        description='Type of the robot'
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_controller), "config", "visualize_ocs2.rviz")

    return LaunchDescription([
        robot_type_arg,
        OpaqueFunction(function=launch_setup),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        )
    ])
