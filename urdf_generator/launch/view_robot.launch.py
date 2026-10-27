# Copyright 2026. Licensed under the Apache License, Version 2.0.
"""Show a generated robot in RViz with joint sliders.

    ros2 launch urdf_generator view_robot.launch.py model:=/path/to/robot.xacro

`model` may be a .xacro or a plain .urdf file (xacro passes plain URDF through).
Pass `gui:=false` to publish zero joint states instead of showing sliders.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('urdf_generator'), 'rviz', 'urdf.rviz')

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')

    # The quotes keep a path that contains spaces in one piece.
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' "', model, '"']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', description='Path to the generated .xacro or .urdf file'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show joint_state_publisher_gui sliders'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui)),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui)),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'),
    ])
