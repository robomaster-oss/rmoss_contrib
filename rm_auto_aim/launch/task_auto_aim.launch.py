import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rm_auto_aim',
            node_executable='task_auto_aim',
            parameters=[
                {'cam_topic_name': 'my_cam/image_raw'},
            ],
            output='screen')
    ])
