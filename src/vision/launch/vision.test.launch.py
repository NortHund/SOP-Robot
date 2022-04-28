import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package="opencv_cam",
        executable="opencv_cam_main",
        arguments=[
            {
                "index": 0,
            }
        ],
    )

    vision_node = Node(
        package="vision",
        executable="vision_node",
        namespace="vision",
        parameters=[
            {
                "classifier": "fer_2013.h5",
                "image_topic": "/image_raw",
            }
        ],
    )

    return LaunchDescription([vision_node, camera_node])