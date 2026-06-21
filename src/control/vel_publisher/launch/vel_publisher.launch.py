from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vel_publisher',
            executable='vel_publisher.py',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
