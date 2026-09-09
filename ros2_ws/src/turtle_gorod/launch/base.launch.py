from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


ARDUINO_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_97565498126230836400-if00"


def generate_launch_description():
    share = get_package_share_directory("turtle_gorod")
    params = os.path.join(share, "config", "base.yaml")

    serial_port = LaunchConfiguration("serial_port")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value=ARDUINO_PORT,
            description="Arduino Uno lower-level controller persistent serial port",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="cmd_vel",
            description="Topic consumed by lower-level bridge",
        ),
        Node(
            package="turtle_gorod",
            executable="serial_bridge",
            name="serial_bridge",
            output="screen",
            parameters=[
                params,
                {
                    "port": serial_port,
                    "cmd_vel_topic": cmd_vel_topic,
                },
            ],
        ),
    ])
