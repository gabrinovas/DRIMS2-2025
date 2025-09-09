from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera1',
            namespace='camera1',
            parameters=[{'serial_no': '243322070068'}],  # as string
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera2',
            namespace='camera2',
            parameters=[{'serial_no': '243322072721'}],  # as string
        ),
    ])
