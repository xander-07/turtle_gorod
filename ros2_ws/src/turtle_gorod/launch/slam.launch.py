from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    turtle_share = get_package_share_directory("turtle_gorod")
    slam_share = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(turtle_share, "config", "slam_toolbox.yaml")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "slam_params_file": slam_params,
                "use_sim_time": "false",
            }.items(),
        )
    ])
