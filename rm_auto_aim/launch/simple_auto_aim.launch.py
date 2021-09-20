import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # camera node
    pkg_rm_auto_aim = get_package_share_directory('rm_auto_aim')
    image_path = os.path.join(pkg_rm_auto_aim, 'resource', 'test.jpg')
    cam_node = Node(
        package='rm_cam',
        executable='virtual_cam',
        name="virtual_image_cam",
        parameters=[{'image_path': image_path,
                     'camera_name': 'front_camera',
                     'camera_k': [1552.7, 0.0, 640.0, 0.0, 1537.9, 360.0, 0.0, 0.0, 1.0],
                     'camera_d': [0.0, 0.0, 0.0, 0.0, 0.0],
                     'camera_p': [0.0, 0.0, 1.0, 0.0,
                                  -1.0, 0.0, 0.0, 0.0,
                                  0.0, -1.0, 0.0, 0.0],
                     'fps': 30}],
        output='screen'
    )
    # simple auto aim node
    params_file = os.path.join(pkg_rm_auto_aim, 'config', 'simple_auto_aim_params.yaml')
    auto_aim_node=Node(
        package='rm_auto_aim',
        executable='simple_auto_aim',
        parameters=[params_file],
        output='screen'
    )
    # add nodes
    ld = LaunchDescription()
    ld.add_action(cam_node)
    ld.add_action(auto_aim_node)
    return ld
