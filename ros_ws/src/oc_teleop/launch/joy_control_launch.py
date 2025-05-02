from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Joystick driver node
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
            ),
            # Custome Teleop Twist Joy node
            Node(
                package="oc_teleop",
                executable="joy_control_node",
                name="joy_control_node",
                output="screen",
            ),
        ]
    )
