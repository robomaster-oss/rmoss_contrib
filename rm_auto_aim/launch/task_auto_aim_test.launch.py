import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #cam_node
    image_name="test.jpg"
    image_path = os.path.join(get_package_share_directory('rm_auto_aim'), 'res/',image_name)
    cam_node=Node(package='rm_cam',executable='virtual_image_cam',name="virtual_cam",
            parameters=[
                {'cam_topic_name': 'virtual_cam/image_raw'},
                {'image_path': image_path},
                {'cam_fps': 30}
            ],
            output='screen'
    )
    #task auto aim node
    auto_aim_params_file = os.path.join(get_package_share_directory('rm_auto_aim'),'res/task_auto_aim_config.yaml')
    auto_aim_node=Node(package='rm_auto_aim',executable='auto_aim_server',
            parameters=[
                {'cam_topic_name': 'virtual_cam/image_raw'},
                auto_aim_params_file
            ],
            output='screen'
    )
    #add node
    ld = LaunchDescription()
    ld.add_action(cam_node)
    ld.add_action(auto_aim_node)
    return ld
