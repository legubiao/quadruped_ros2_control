import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def launch_setup(context, *args, **kwargs):
    # Gazebo World
    world = context.launch_configurations['world']
    default_sdf_path = os.path.join(get_package_share_directory('gz_quadruped_playground'), 'worlds', world + '.sdf')
    print(default_sdf_path)

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
        'GAZEBO': 'true',
        'EXTERNAL_SENSORS': 'true'
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

    # Controllers
    controller = context.launch_configurations['controller']
    if controller == 'ocs2':
        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('gz_quadruped_playground'),
                                                                 'launch',
                                                                 'ocs2.launch.py'])])
        )
        rviz_config_file = os.path.join(get_package_share_directory('gz_quadruped_playground'), "config", "ocs2.rviz")
    else:
        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('gz_quadruped_playground'),
                                                                 'launch',
                                                                 'unitree_guide.launch.py'])])
        )
        rviz_config_file = os.path.join(get_package_share_directory('gz_quadruped_playground'), "config", "rviz.rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=["-d", rviz_config_file]
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
        controller_launch
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

    controller = DeclareLaunchArgument(
        'controller',
        default_value='unitree_guide',
        description='The ROS2-Control Controllers'
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/rgbd_d435/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            # "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            # "/odom_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance",
            # "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
            '/rgbd_d435/depth_image',
            '/rgbd_d435/image',
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True,
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    return LaunchDescription([
        world,
        pkg_description,
        height,
        controller,
        gz_bridge_node,
        gz_image_bridge_node,
        OpaqueFunction(function=launch_setup),
    ])
