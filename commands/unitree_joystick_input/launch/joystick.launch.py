from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_joystick_input',
            executable='joystick_input',
            name='joystick_input_node',
            parameters=[
                {
                    'network_interface': 'enp46s0',
                }
            ],
        )
    ])
