# Copyright 2026 User
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""
Launch OpenArm skill server with one of three stack modes:

  use_demo:=true              MoveIt demo stack (default, unchanged behaviour)
  use_gravity_comp:=true      MoveIt + gravity-compensated hardware feedforward
  both false                  skill_server only (stack already running elsewhere)

use_demo and use_gravity_comp are mutually exclusive.

remote_rviz:=true             Board-side sim / remote-view: fake hardware, no local
                              RViz. On another PC (same ROS_DOMAIN_ID):
                              ros2 launch openarm_skills remote_viewer.launch.py \\
                                use_fake_hardware:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
            description="Launch demo.launch.py (MoveIt + JTC, no gravity feedforward).",
        ),
        DeclareLaunchArgument(
            "use_gravity_comp",
            default_value="false",
            description="Launch demo_gravity.launch.py (MoveIt + KDL gravity feedforward).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz on this machine (ignored when remote_rviz:=true).",
        ),
        DeclareLaunchArgument(
            "remote_rviz",
            default_value="false",
            description=(
                "Sim + remote RViz mode on this machine: use_fake_hardware:=true, "
                "use_rviz:=false. Run remote_viewer.launch.py on the viewer PC."
            ),
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake ros2_control hardware (forced true when remote_rviz:=true).",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="Passed to the selected MoveIt launch file.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="Passed to the selected MoveIt launch file.",
        ),
        DeclareLaunchArgument(
            "gravity_scale",
            default_value="1.0",
            description="Gravity feedforward scale (gravity stack only).",
        ),
        DeclareLaunchArgument(
            "skill_startup_delay_s",
            default_value="8.0",
            description="Seconds to wait for stack before skill_server starts.",
        ),
        DeclareLaunchArgument(
            "openarm_debug",
            default_value="false",
            description="Set skill_server log level to debug",
        ),
    ]

    skill_log_args = [
        "--ros-args",
        "--log-level",
        PythonExpression([
            "'skill_server:=debug' if '",
            LaunchConfiguration("openarm_debug"),
            "' == 'true' else 'skill_server:=info'",
        ]),
        "--log-level", "rcl:=warn",
        "--log-level", "rmw_fastrtps_cpp:=warn",
        "--log-level", "rclcpp:=info",
    ]

    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    skill_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        cfg,
    ]

    skill_server_node = Node(
        package="openarm_skills",
        executable="skill_server_node",
        name="skill_server",
        output="screen",
        parameters=skill_params,
        arguments=skill_log_args,
    )

    demo_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("use_demo"),
                "' == 'true' and '",
                LaunchConfiguration("use_gravity_comp"),
                "' != 'true'",
            ]
        )
    )

    gravity_condition = IfCondition(LaunchConfiguration("use_gravity_comp"))

    stack_only_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("use_demo"),
                "' != 'true' and '",
                LaunchConfiguration("use_gravity_comp"),
                "' != 'true'",
            ]
        )
    )

    stack_with_delay_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("use_demo"),
                "' == 'true' or '",
                LaunchConfiguration("use_gravity_comp"),
                "' == 'true'",
            ]
        )
    )

    shared_launch_args = {
        "use_rviz": PythonExpression([
            "'false' if '",
            LaunchConfiguration("remote_rviz"),
            "' == 'true' else '",
            LaunchConfiguration("use_rviz"),
            "'",
        ]),
        "use_fake_hardware": PythonExpression([
            "'true' if '",
            LaunchConfiguration("remote_rviz"),
            "' == 'true' else '",
            LaunchConfiguration("use_fake_hardware"),
            "'",
        ]),
        "right_can_interface": LaunchConfiguration("right_can_interface"),
        "left_can_interface": LaunchConfiguration("left_can_interface"),
    }

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("openarm_bimanual_moveit_config"),
            "/launch/demo.launch.py",
        ]),
        launch_arguments=shared_launch_args.items(),
        condition=demo_condition,
    )

    gravity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("openarm_bimanual_moveit_config"),
            "/launch/demo_gravity.launch.py",
        ]),
        launch_arguments={
            **shared_launch_args,
            "gravity_scale": LaunchConfiguration("gravity_scale"),
        }.items(),
        condition=gravity_condition,
    )

    delayed_skill_server = TimerAction(
        period=LaunchConfiguration("skill_startup_delay_s"),
        actions=[skill_server_node],
        condition=stack_with_delay_condition,
    )

    return LaunchDescription(
        declared_arguments
        + [
            demo_launch,
            gravity_launch,
            delayed_skill_server,
            Node(
                package="openarm_skills",
                executable="skill_server_node",
                name="skill_server",
                output="screen",
                parameters=skill_params,
                arguments=skill_log_args,
                condition=stack_only_condition,
            ),
        ]
    )
