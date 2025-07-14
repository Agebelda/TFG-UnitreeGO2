from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_vr',
            executable='stereo_publisher',
            name='stereo_publisher',
            output='screen'
        ),
        Node(
            package='go2_vr',
            executable='cmd_vel_request',
            name='cmd_vel_request',
            output='screen'
        )
    ])
