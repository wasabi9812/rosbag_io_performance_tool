from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    pkg = 'robot_image_publisher'
    config_path = os.path.join(
        get_package_share_directory(pkg), 'config', 'robot_image_config.yaml'
    )
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    p = cfg['robot_image']
    robot_count   = int(p.get('robot_count', 1))
    start_index   = int(p.get('start_index', 0))
    publish_rate  = float(p.get('publish_rate', 1.0))
    message_count = p.get('message_count', "no")   # "no" 또는 정수 문자열
    width         = int(p.get('width', 640))
    height        = int(p.get('height', 480))
    jpeg_quality  = int(p.get('jpeg_quality', 85))
    pattern       = str(p.get('pattern', "moving"))
    frame_id      = str(p.get('frame_id', "camera"))

    nodes = []
    for i in range(start_index, start_index + robot_count):
        nodes.append(
            Node(
                package=pkg,                         # ✅ 고침
                executable='robot_image_publisher',  # ✅ setup.py의 entry_points와 동일
                name=f'robot_image_publisher_{i}',
                output='screen',
                parameters=[{
                    "robot_index": i,
                    "publish_rate": publish_rate,
                    "message_count": message_count,
                    "width": width,
                    "height": height,
                    "jpeg_quality": jpeg_quality,
                    "pattern": pattern,
                    "frame_id": frame_id
                }]
            )
        )
    return LaunchDescription(nodes)
