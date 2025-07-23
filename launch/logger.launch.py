import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('bopa_logger')
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='bopa_logger',
            executable='run.py',
            output='screen',
            parameters=[config_file],
        )
    ])