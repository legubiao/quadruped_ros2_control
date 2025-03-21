import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

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

    elevation_mapping = Node (
        package="elevation_mapping",
        executable="elevation_mapping",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("ocs2_quadruped_controller"),
                    "config",
                    "elevation_mapping.yaml",
                ]
            )
        ],
    )

    elevation_mapping = Node (
        package="elevation_mapping",
        executable="elevation_mapping",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("ocs2_quadruped_controller"),
                    "config",
                    "elevation_mapping.yaml",
                ]
            )
        ],
    )

    convex_plane_decomposition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("convex_plane_decomposition_ros"),
                "launch",
                "convex_plane_decomposition.launch.py"
            ])
        ]),
        launch_arguments={
            "parameter_file": PathJoinSubstitution([
                FindPackageShare("legged_perceptive_controllers"),
                "config",
                "convex_plane_decomposition.yaml"
            ])
        }.items()
    )


    return LaunchDescription([
        joint_state_publisher,
        # elevation_mapping,
        # convex_plane_decomposition,
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
    ])
