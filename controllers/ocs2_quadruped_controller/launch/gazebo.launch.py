import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, \
    TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Gazebo World
    world = context.launch_configurations['world']
    default_sdf_path = os.path.join(get_package_share_directory('gz_quadruped_playground'), 'worlds', world + '.sdf')

    # Init Height When spawn the model
    init_height = context.launch_configurations['height']
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'robot', '-allow_renaming', 'true', '-z', init_height],
    )

    # Robot Description
    pkg_description = context.launch_configurations['pkg_description']
    pkg_path = os.path.join(get_package_share_directory(pkg_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        'GAZEBO': 'true'
    }).toxml()
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

    rviz_config_file = os.path.join(get_package_share_directory("ocs2_quadruped_controller"), "config", "visualize_ocs2.rviz")
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"]
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"]
    )

    ocs2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_quadruped_controller", "--controller-manager", "/controller_manager"]
    )
    
    return [
        rviz,
        robot_state_publisher,
        gz_spawn_entity,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 ', default_sdf_path])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_publisher,imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[ocs2_controller],
            )
        ),
    ]


def generate_launch_description():
    world = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='The world to load'
    )

    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )

    height = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Init height in simulation'
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ],
        output="screen"
    )

    return LaunchDescription([
        world,
        pkg_description,
        height,
        gz_bridge_node,
        OpaqueFunction(function=launch_setup),
    ])
