from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


YDLIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"


def generate_launch_description():
    share = get_package_share_directory("turtle_gorod")
    params = os.path.join(share, "config", "ydlidar_x3.yaml")

    lidar_port = LaunchConfiguration("lidar_port")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value=YDLIDAR_PORT,
            description="YDLIDAR X3 persistent serial port",
        ),
        # base_link: X forward, Y left, Z up.
        # LiDAR is centered laterally, mounted about 4 cm behind the robot
        # center and the scan plane is about 13 cm above the floor.
        DeclareLaunchArgument("lidar_x", default_value="-0.04"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.13"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            parameters=[params, {"port": lidar_port}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_static_tf",
            output="screen",
            arguments=[
                "--x", lidar_x,
                "--y", lidar_y,
                "--z", lidar_z,
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", lidar_yaw,
                "--frame-id", "base_link",
                "--child-frame-id", "laser_frame",
            ],
        ),
    ])
