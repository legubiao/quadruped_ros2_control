import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name = context.launch_configurations['robot']
    
    # 自动补上_description后缀
    package_description = f"{robot_name}_description"
    
    try:
        pkg_path = os.path.join(get_package_share_directory(package_description))
    except Exception as e:
        print(f"Error: Could not find package '{package_description}'. Please ensure it is installed.")
        print(f"Available packages can be checked with: ros2 pkg list | grep {robot_name}")
        raise e

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    
    if not os.path.exists(xacro_file):
        print(f"Error: Could not find robot description file: {xacro_file}")
        raise FileNotFoundError(f"Robot description file not found: {xacro_file}")
    
    print(f"Using robot description file: {xacro_file}")
    
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control.yaml",
        ]
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_description), "config", "visualize_urdf.rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
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
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
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

    basic_quadruped_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["basic_quadruped_controller", "--controller-manager", "/controller_manager"],
    )

    return [
        rviz,
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
                on_exit=[basic_quadruped_controller],
            )
        ),
    ]


def generate_launch_description():
    robot_name = DeclareLaunchArgument(
        'robot',
        default_value='unitree_go2',
        description='name of the robot (e.g., go2, go1)'
    )

    return LaunchDescription([
        robot_name,
        OpaqueFunction(function=launch_setup),
    ])
