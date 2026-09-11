from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("turtle_gorod")
    slam_launch = os.path.join(share, "launch", "slam.launch.py")
    rviz_config = os.path.join(share, "config", "mapping.rviz")

    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 with mapping view",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_mapping",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
