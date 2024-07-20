#!/usr/bin/env python3
#
# Author: Placido Falqueto placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id', default='')

    robot_name = PythonExpression(["'", 'limo', robot_id, "'"])
    frame_prefix = PythonExpression(["'", 'limo', robot_id, "/'"])

    xacro_model = os.path.join(
        get_package_share_directory('limo_description'),
        'urdf', 'limo_four_diff.xacro')

    robot_desc = Command(['xacro ', xacro_model, ' robot_id:=', robot_id])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Robot ID'),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'frame_prefix': frame_prefix,
                         'robot_description': robot_desc,
                         'use_sim_time': True}],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'frame_prefix': frame_prefix,
                         'robot_description': robot_desc,
                         'use_sim_time': True}],
        )
    ])
