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
    limo_nav2_pkg = get_package_share_directory('limo_navigation')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    project_pkg = os.path.join(get_package_share_directory('project'))

    nav2_params_file_path = os.path.join(
        limo_nav2_pkg, 'config', 'limo.yaml')

    # General arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    n_robot = LaunchConfiguration('n_robot', default='4')
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file', default=os.path.join(project_pkg, 'rviz', 'limo.rviz'))

    # Gazebo simulation arguments
    use_gui = LaunchConfiguration('use_gui', default='true')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(
        limo_desc_pkg, 'worlds', 'empty.world'))

    # Navigation arguments
    map_file = LaunchConfiguration('map_file', default=os.path.join(
        limo_nav2_pkg, 'maps', 'hexagon.yaml'))
    nav2_params_file = LaunchConfiguration(
        'nav2_params_file', default=nav2_params_file_path)
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(
        limo_nav2_pkg, 'rviz', 'limo_nav.rviz'))

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': gazebo_world_file,
                'verbose': 'true'
            }.items(),
        ),

        # Navigation arguments
        DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=nav2_params_file_path,
            description='Full path to the nav2 params file to use'
        ),
        DeclareLaunchArgument(
            'nav2_rviz_config_file',
            default_value=nav2_rviz_config_file,
            description='Full path to the nav2 rviz config file to use'
        ),
        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RVIZ config file to use'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={
                'verbose': 'true'
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

    ]

    def launch_rsp(context):
        nodes = []
        for robot_id in range(int(context.launch_configurations['n_robot'])):

            rsp_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(limo_desc_pkg, 'launch'),
                    '/rsp.launch.py']
                ),
                launch_arguments={
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

    def launch_navigation(context):
        nodes = []
        for robot_id in range(int(context.launch_configurations['n_robot'])):

            sapf_node = GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(limo_nav2_pkg, 'launch'),
                        '/limo_nav.launch.py']
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'robot_id': str(robot_id),
                        'map_file': map_file,
                        'nav2_params_file': nav2_params_file,
                        'rviz_config_file': nav2_rviz_config_file,
                        'initial_x': str(robot_id*2.0-2.0),
                        'initial_y': str(robot_id*2.0-2.0),
                        'headless': 'true',
                    }.items()
                ),
            ])

            nodes.append(sapf_node)

        return nodes

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=launch_rsp))
    ld.add_action(OpaqueFunction(function=spawn_robot))

    ld.add_action(TimerAction(
        period='5',
        actions=[OpaqueFunction(function=launch_navigation)]
    ))

    return ld
