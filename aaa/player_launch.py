from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aaa',
            executable='aaa',
            arguments=['aaa_1']
        ),
        Node(
            package='aaa',
            executable='aaa',
            arguments=['aaa_2']
        )
    ])
