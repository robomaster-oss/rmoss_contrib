import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_name="test.png"
    image_path = os.path.join(get_package_share_directory('rm_auto_aim'), 'res/',image_name)
    return LaunchDescription([
        Node(package='rm_cam',
            node_executable='task_sim_cam_image',
            parameters=[
                {'cam_topic_name': 'my_cam/image_raw'},
                {'image_path': image_path},
                {'cam_fps': 20}
            ],
            output='screen')
    ])
