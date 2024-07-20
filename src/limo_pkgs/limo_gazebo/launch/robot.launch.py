#!/usr/bin/env python3
#
# Author: Placido Falqueto   placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import logging
import launch.logging


def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    limo_desc_pkg = get_package_share_directory('limo_description')
    limo_gaze_pkg = get_package_share_directory('limo_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_id = LaunchConfiguration('robot_id', default='')
    use_gui = LaunchConfiguration('use_gui', default='true')
    spawn_limo = LaunchConfiguration('spawn_limo', default='true')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file',
                                            default=os.path.join(
                                                limo_gaze_pkg, 'worlds', 'empty.world')
                                            )

    robot_name = PythonExpression(["'", 'limo', robot_id, "'"])

    # TODO move models to limo_gazebo
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(limo_desc_pkg, 'models')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value=use_sim_time,
            description='Flag to enable use of simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            name='robot_id',
            default_value=robot_id,
            description='ID of the robot'
        ),
        DeclareLaunchArgument(
            name='use_gui',
            default_value=use_gui,
            choices=['true', 'false'],
            description='Flag to enable gazebo visualization'
        ),
        DeclareLaunchArgument(
            name='spawn_limo',
            default_value=spawn_limo,
            choices=['true', 'false'],
            description='Flag to enable spawning of the robot'
        ),
        DeclareLaunchArgument(
            name='gazebo_world_file',
            default_value=gazebo_world_file,
            description='World used in the gazebo simulation'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': gazebo_world_file,
                'verbose': 'true'
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={
                'verbose': 'true'
            }.items(),
            condition=IfCondition(use_gui)
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', PythonExpression(["'/", robot_name,
                                           "/robot_description", "'"]),
                '-entity', robot_name,
                '-robot_namespace', robot_name]
        )

    ])
