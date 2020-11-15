import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #virtual cam node
    video_path="/home/ubuntu/project/rmoss_ws/src/res/power_rune2019_red_demo.avi"
    cam_node=Node(package='rm_cam',executable='virtual_video_cam',name="virtual_cam",
            parameters=[
                {'cam_topic_name': 'virtual_cam/image_raw'},
                {'video_path': video_path},
            ],
            output='screen'
    )
    #task auto aim node
    task_power_rune_params_file = os.path.join(get_package_share_directory('rm_power_rune2019'),'res/task_power_rune_config.yaml')
    task_power_rune_node=Node(package='rm_power_rune2019',executable='task_power_rune',name="task_power_rune",
            parameters=[
                {'cam_topic_name': 'virtual_cam/image_raw'},
                task_power_rune_params_file
            ],
            output='screen'
    )
    #add node
    ld = LaunchDescription()
    ld.add_action(cam_node)
    ld.add_action(task_power_rune_node)
    return ld
