from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(name="config", default_value="params.yml", description="Input config filename"),
        DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"),
    ]
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
    config_filename = PathJoinSubstitution([FindPackageShare('face_turners'), 'config', LaunchConfiguration("config")]).perform(context)

    with open(config_filename, 'r') as f:
        config = yaml.safe_load(f)

    topics_cf = config['topics']

    gui_node = Node(
        package="face_turners", 
        executable="gui_node",
        name="gui",
        output="screen",
        parameters=[
            {"img_topic": topics_cf["img_topic"]},
            {"dice_detection_topic": topics_cf["dice_detection_topic"]},
            {"use_sim_time": LaunchConfiguration("fake")}
        ],
    )

    return [
        gui_node
    ]