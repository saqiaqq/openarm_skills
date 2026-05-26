# Copyright 2026 User
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Remote RViz viewer for openArm — thin wrapper.

Delegates entirely to openarm_bimanual_moveit_config/launch/remote_rviz.launch.py
so that URDF / SRDF / kinematics changes only need to be maintained in one place.

Usage (on viewer machine, same LAN, same ROS_DOMAIN_ID as robot):
    export ROS_DOMAIN_ID=10
    ros2 launch openarm_skills remote_viewer.launch.py

Optional overrides (forwarded to remote_rviz.launch.py):
    ros2 launch openarm_skills remote_viewer.launch.py use_fake_hardware:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time",       default_value="false"),
        DeclareLaunchArgument("description_package", default_value="openarm_description"),
        DeclareLaunchArgument("description_file",    default_value="v10.urdf.xacro"),
        DeclareLaunchArgument("arm_type",            default_value="v10"),
        DeclareLaunchArgument("use_fake_hardware",   default_value="false"),
        DeclareLaunchArgument("use_sim_hardware",    default_value="false"),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface",  default_value="can1"),
    ]

    remote_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("openarm_bimanual_moveit_config"),
            "/launch/remote_rviz.launch.py",
        ]),
        launch_arguments={
            "use_sim_time":        LaunchConfiguration("use_sim_time"),
            "description_package": LaunchConfiguration("description_package"),
            "description_file":    LaunchConfiguration("description_file"),
            "arm_type":            LaunchConfiguration("arm_type"),
            "use_fake_hardware":   LaunchConfiguration("use_fake_hardware"),
            "use_sim_hardware":    LaunchConfiguration("use_sim_hardware"),
            "right_can_interface": LaunchConfiguration("right_can_interface"),
            "left_can_interface":  LaunchConfiguration("left_can_interface"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [remote_rviz])
