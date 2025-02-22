import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
def generate_launch_description():
    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pd_controller",
                   "--controller-manager", "/controller_manager"],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_guide_controller", "--controller-manager", "/controller_manager"],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        leg_pd_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[imu_sensor_broadcaster, joint_state_publisher, unitree_guide_controller],
            )
        ),
    ])
