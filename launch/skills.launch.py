"""Launch OpenArm skill server, optionally with MoveIt demo stack.

Default (use_demo:=true): starts demo.launch.py (controllers + move_group + RViz)
then skill_server_node after a short delay so MoveIt is ready.

Skills-only (use_demo:=false): only skill_server_node — same as the old workflow
when demo is already running in another terminal.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("openarm_skills"), "config", "skills.yaml"
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "use_demo",
            default_value="true",
            description="If true, also launch openarm_bimanual_moveit_config demo.launch.py",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Passed to demo.launch.py when use_demo is true",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Passed to demo.launch.py (true = no real CAN motors)",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="Passed to demo.launch.py",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="Passed to demo.launch.py",
        ),
        DeclareLaunchArgument(
            "skill_startup_delay_s",
            default_value="8.0",
            description="Seconds to wait for demo/controllers before skill_server starts",
        ),
    ]

    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    skill_server_node = Node(
        package="openarm_skills",
        executable="skill_server_node",
        name="skill_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            cfg,
        ],
    )

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("openarm_bimanual_moveit_config"),
            "/launch/demo.launch.py",
        ]),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "right_can_interface": LaunchConfiguration("right_can_interface"),
            "left_can_interface": LaunchConfiguration("left_can_interface"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_demo")),
    )

    delayed_skill_server = TimerAction(
        period=LaunchConfiguration("skill_startup_delay_s"),
        actions=[skill_server_node],
        condition=IfCondition(LaunchConfiguration("use_demo")),
    )

    return LaunchDescription(
        declared_arguments
        + [
            demo_launch,
            delayed_skill_server,
            Node(
                package="openarm_skills",
                executable="skill_server_node",
                name="skill_server",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    cfg,
                ],
                condition=UnlessCondition(LaunchConfiguration("use_demo")),
            ),
        ]
    )
