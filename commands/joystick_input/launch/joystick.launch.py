from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joynode',
            parameters=[{
                'dev': '/dev/input/js0'
            }]
        ),
        Node(
            package='joystick_input',
            executable='joystick_input',
            name='joystick_input_node'
        )
    ])
