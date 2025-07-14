from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_server',
            output='screen',
            parameters=[{'ROS_IP': '192.168.123.75'}]
        ),
        Node(
            package='go2_vr',
            executable='image_compressor_left',
            name='image_compressor_left',
            output='screen'
        ),
        Node(
            package='go2_vr',
            executable='image_compressor_right',
            name='image_compressor_right',
            output='screen'
        )
    ])
