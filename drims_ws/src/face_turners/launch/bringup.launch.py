from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(name="robot_name", default_value="ur10e", description="Robot name"),
        DeclareLaunchArgument(name="robot_ip", default_value="192.168.254.100", description="Robot ip"),
        DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"),
    ]
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    launch_robot_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', robot_name, f'{robot_name}_start.launch.py'])
    launch_robot_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(launch_robot_path),
        launch_arguments = [
        ('fake', LaunchConfiguration("fake")),
        ('robot_ip', LaunchConfiguration("robot_ip"))
        ]
    )

    return [
        launch_robot_launch,
    ]