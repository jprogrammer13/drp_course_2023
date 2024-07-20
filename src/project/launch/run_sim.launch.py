#!/usr/bin/env python3
#
# Author: Enrico Saccon  enrico.saccon [at] unitn.it

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

import launch.logging
import logging


def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG

    limo_desc_pkg = get_package_share_directory('limo_description')
    limo_gaze_pkg = get_package_share_directory('limo_gazebo')

    # General arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    n_robot = LaunchConfiguration('n_robot', default='4')

    # Gazebo simulation arguments
    use_gui = LaunchConfiguration('use_gui', default='true')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(
        limo_gaze_pkg, 'worlds', 'empty.world'))

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        # General arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'n_robot',
            default_value=n_robot,
            description='The number of robot to spawn'
        ),

        # Gazebo simulation arguments
        DeclareLaunchArgument(
            'use_gui',
            default_value=use_gui,
            choices=['true', 'false'],
            description='Flag to enable gazebo visualization'
        ),
        DeclareLaunchArgument(
            'gazebo_world_file',
            default_value=gazebo_world_file,
            description='World used in the gazebo simulation'
        ),

    ]

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(limo_gaze_pkg, 'launch'),
            '/robot.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'spawn_limo': 'false',
            'use_gui': use_gui,
            'gazebo_world_file': gazebo_world_file
        }.items()
    )

    def launch_rsp(context):
        nodes = []
        for robot_id in range(int(context.launch_configurations['n_robot'])):

            robot_name = PythonExpression(
                ["'", "limo", str(robot_id), "'"])

            rsp_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(limo_desc_pkg, 'launch'),
                    '/rsp.launch.py']
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_id': str(robot_id),
                }.items()
            )
            nodes.append(rsp_node)

        return nodes

    def spawn_robot(context):
        nodes = []
        for robot_id in range(int(context.launch_configurations['n_robot'])):

            robot_name = PythonExpression(
                ["'", "limo", str(robot_id), "'"])

            spawn_node = GroupAction([
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                            '-topic', PythonExpression(
                                ["'/", robot_name, "/robot_description", "'"]),
                            '-entity', robot_name,
                            '-robot_namespace', robot_name,
                            '-x', str(robot_id*2.0-2.0),
                            '-y', str(robot_id*2.0-2.0),
                            '-z', '0.0',
                            '-Y', '0.0',
                    ]
                ),
            ])
            nodes.append(spawn_node)

        return nodes

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(gazebo_launch)

    ld.add_action(OpaqueFunction(function=launch_rsp))
    ld.add_action(OpaqueFunction(function=spawn_robot))

    return ld