from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the Hikvision camera configuration file
    hik_config_path = os.path.join(
        get_package_share_directory('camera_aravis2'),
        'config',
        'hik_config.yaml'
    )
    
    return LaunchDescription([
        # Launch Realsense Camera 1 node with its serial number and namespace
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera1',
            namespace='camera1',
            parameters=[{'serial_no': '243322070068'}],
        ),
        # Launch Realsense Camera 2 node with its serial number and namespace
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera2',
            namespace='camera2',
            parameters=[{'serial_no': '243322072721'}],
        ),
        # Launch Hikvision camera node with external configuration file
        Node(
            package='camera_aravis2',
            executable='camera_aravis2_node',
            name='hik_camera',
            namespace='hik_camera',
            parameters=[hik_config_path],
        ),
    ])