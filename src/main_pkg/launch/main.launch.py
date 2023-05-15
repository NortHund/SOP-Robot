from launch import LaunchDescription
from launch_ros.actions import Node


package_name = "main_pkg"




def generate_launch_description():
    
    launch_description = LaunchDescription()


    face_recognition_node = Node(
        package=package_name,
        executable="face_recognition",
    )

    emotion_detection_node = Node(
        package=package_name,
        executable="emotion_detection",

    )

    action_client_node = Node(
        package=package_name,
        executable="action_client",
        # output='screen',
        # emulate_tty=True,
    )


    launch_description.add_action(face_recognition_node)
    launch_description.add_action(emotion_detection_node)
    launch_description.add_action(action_client_node)

    return launch_description