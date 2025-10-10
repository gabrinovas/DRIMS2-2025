from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch two Intel RealSense D435i camera nodes, each with a unique serial number and namespace
    return LaunchDescription([
        # First RealSense camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera1',
            namespace='camera1',
            parameters=[{'serial_no': '243322070068'}],  # Serial number as string
        ),
        # Second RealSense camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera2',
            namespace='camera2',
            parameters=[{'serial_no': '243322072721'}],  # Serial number as string
        ),
    ])
