from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("turtle_gorod")
    slam_launch = os.path.join(share, "launch", "slam.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
        ),
        Node(
            package="turtle_gorod",
            executable="map_web",
            name="map_web",
            output="screen",
            parameters=[{"port": 8080}],
        ),
    ])
