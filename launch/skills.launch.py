"""Launch the OpenArm skill server.

This launch file ONLY brings up the skill_server node.  The MoveIt move_group,
controllers and (optionally) the perception node must already be running -- the
recommended way is to first launch
    ros2 launch openarm_bimanual_moveit_config demo.launch.py
and then this file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("openarm_skills"), "config", "skills.yaml"
    )
    return LaunchDescription([
        Node(
            package="openarm_skills",
            executable="skill_server_node",
            name="skill_server",
            output="screen",
            parameters=[cfg],
        )
    ])
