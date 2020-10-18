import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    #cam_node
    image_name="test.jpg"
    image_path = os.path.join(get_package_share_directory('rm_auto_aim'), 'res/',image_name)
    cam_node=Node(package='rm_cam',executable='task_sim_cam_image',name="sim_camera",
            parameters=[
                {'cam_topic_name': 'sim_camera/image_raw'},
                {'image_path': image_path},
                {'cam_fps': 30}
            ],
            output='screen'
    )
    #task auto aim node
    auto_aim_params = os.path.join(get_package_share_directory('rm_auto_aim'),'res/task_auto_aim_config.yaml')
    auto_aim_node=Node(package='rm_auto_aim',executable='task_auto_aim',name="task_auto_aim",
            parameters=[
                {'cam_topic_name': 'sim_camera/image_raw'},
                auto_aim_params
            ],
            output='screen'
    )
    #add node
    ld.add_action(cam_node)
    ld.add_action(auto_aim_node)
    return ld
