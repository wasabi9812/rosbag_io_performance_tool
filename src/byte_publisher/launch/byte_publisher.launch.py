from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('byte_publisher'),
        'config',
        'byte_publisher_config.yaml'
    )
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)['byte_publisher']

    return LaunchDescription([
        Node(
            package='byte_publisher',
            executable='byte_publisher',
            name='byte_publisher',
            output='screen',
            parameters=[cfg],
        )
    ])
