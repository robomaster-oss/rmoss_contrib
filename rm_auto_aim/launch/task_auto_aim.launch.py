import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rm_auto_aim',
            node_executable='task_auto_aim',
            parameters=[
                {'usb_cam_path': '/dev/video0'},
                {'cam_fps': 20}
            ],
            output='screen')
    ])
