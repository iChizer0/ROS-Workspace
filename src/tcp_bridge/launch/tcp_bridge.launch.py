from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tcp_bridge',
            namespace='tcp_bridge',
            executable='bridge',
            name='bridge'
        )
    ])
