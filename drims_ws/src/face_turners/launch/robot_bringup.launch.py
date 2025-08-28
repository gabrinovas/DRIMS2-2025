from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

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

    robot_cf = config['robot']

    robot_name = robot_cf['robot_name']
    launch_robot_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', robot_name, f'{robot_name}_start.launch.py'])
    launch_robot_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(launch_robot_path),
        launch_arguments = [
        ('fake', LaunchConfiguration("fake")),
        ('robot_ip', robot_cf['robot_ip']),
        ]
    )

    camera_path = PathJoinSubstitution([FindPackageShare("depthai_examples"), "launch", "rgb_stereo_node.launch.py"])
    camera_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(camera_path),
    )

    return [
        launch_robot_launch,
        camera_launch,
    ]