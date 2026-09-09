from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


ARDUINO_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_97565498126230836400-if00"
YDLIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"


def generate_launch_description():
    share = get_package_share_directory("turtle_gorod")

    serial_port = LaunchConfiguration("serial_port")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "base.launch.py")),
        launch_arguments={
            "serial_port": serial_port,
            "cmd_vel_topic": "cmd_vel_safe",
        }.items(),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar.launch.py")),
        launch_arguments={
            "lidar_port": lidar_port,
            "lidar_x": lidar_x,
            "lidar_y": lidar_y,
            "lidar_z": lidar_z,
            "lidar_yaw": lidar_yaw,
        }.items(),
    )

    guard = Node(
        package="turtle_gorod",
        executable="collision_guard",
        name="collision_guard",
        output="screen",
        parameters=[{
            "require_scan": True,
            "stop_distance_m": 0.22,
            "slow_distance_m": 0.38,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value=ARDUINO_PORT),
        DeclareLaunchArgument("lidar_port", default_value=YDLIDAR_PORT),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.10"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        base,
        lidar,
        guard,
    ])
