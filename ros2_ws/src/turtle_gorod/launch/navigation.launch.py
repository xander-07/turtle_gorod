from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    turtle_share = get_package_share_directory("turtle_gorod")
    nav2_share = get_package_share_directory("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params = os.path.join(turtle_share, "config", "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description="Absolute path to a Nav2 map yaml file",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_yaml,
                "params_file": params,
                "use_sim_time": "False",
                "autostart": "True",
                "use_composition": "False",
            }.items(),
        ),
    ])
