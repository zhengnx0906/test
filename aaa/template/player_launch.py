from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='template',
            executable='template',
            arguments=['template_1']
        ),
        Node(
            package='template',
            executable='template',
            arguments=['template_2']
        )
    ])
