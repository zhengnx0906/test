from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='threeidiots',
            executable='threeidiots',
            arguments=['threeidiots_1']
        ),
        Node(
            package='threeidiots',
            executable='threeidiots',
            arguments=['threeidiots_2']
        )
    ])
