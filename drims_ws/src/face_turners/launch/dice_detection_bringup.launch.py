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
    detector_cf = config['detector']
    robot_cf = config['robot']

    camera_cf = config['camera'][robot_cf["robot_id"]]

    dice_detector_node = Node(
        package="face_turners", 
        executable="dice_detector_node",
        name="dice_detector",
        output="screen",
        parameters=[
            {"img_topic":  topics_cf["img_topic"]},
            {"dice_detection_mask_topic": topics_cf["dice_detection_mask_topic"]},
            {"dice_detection_img_topic": topics_cf["dice_detection_img_topic"]},
            {"dice_detection_srv_topic": topics_cf["dice_detection_srv_topic"]},
            {"hue_range": detector_cf["hue_range"]},
            {"saturation_range": detector_cf["saturation_range"]},
            {"value_range": detector_cf["value_range"]},
            {"min_point_area": detector_cf["min_point_area"]},
            {"max_point_area": detector_cf["max_point_area"]},
            {"min_inertia_ratio": detector_cf["min_inertia_ratio"]},
            {"min_circularity": detector_cf["min_circularity"]},
            {"dbscan_eps": detector_cf["dbscan_eps"]},
            {"K": camera_cf["K"]},
            {"d": camera_cf["d"]},
            {"T_c2w": camera_cf["T_c2w"]},
            {"die_size": detector_cf["die_size"]},
            {"base_frame": topics_cf["base_frame"]},
            {"use_sim_time": LaunchConfiguration("fake")}
        ],
    )

    return [
        dice_detector_node
    ]