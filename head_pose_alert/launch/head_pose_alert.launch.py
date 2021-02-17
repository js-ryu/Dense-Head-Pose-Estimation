import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('head_pose_alert'),
        'config',
        'params.yaml'
        )

    node=Node(
        package='head_pose_alert',
        node_executable='head_pose_alert',
        output='screen',
        # The "threshold" is left, right, up, down from the left. 
        parameters = [config]
    )        

    ld.add_action(node)
    return ld

