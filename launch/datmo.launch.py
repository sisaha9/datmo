from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    param_file = os.path.join(get_package_share_directory("datmo"), "param", "datmo.yaml")
    return LaunchDescription(
        [
            Node(
                package="datmo",
                executable="datmo_node_exe",
                name="datmo_node",
                output="screen",
                parameters=[param_file],
                remappings=[
                    ("scan", "/scan"),
                    ("nav", "/nav")
                ],
            ),
        ]
    )
