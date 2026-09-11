from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("turtle_gorod")
    navigation_launch = os.path.join(share, "launch", "navigation.launch.py")

    map_yaml = LaunchConfiguration("map")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description="Absolute path to a Nav2 map yaml file",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="8080",
            description="HTTP port for the browser map viewer",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={"map": map_yaml}.items(),
        ),
        Node(
            package="turtle_gorod",
            executable="map_web",
            name="map_web_navigation",
            output="screen",
            parameters=[{"port": port}],
        ),
    ])
