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


def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return


def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG

    limo_desc_pkg = get_package_share_directory('limo_description')
    # shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    limo_gaze_pkg = get_package_share_directory('limo_gazebo')
    # map_env_pkg = get_package_share_directory('map_pkg')

    # nav2_params_file_path = os.path.join(
    #     shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    # map_env_params_file_path = os.path.join(
    #     map_env_pkg, 'config', 'map_config.yaml')

    # General arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    n_robot = LaunchConfiguration('n_robot', default='4')

    # Gazebo simulation arguments
    use_gui = LaunchConfiguration('use_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(
        limo_desc_pkg, 'rviz', 'limo.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(
        limo_gaze_pkg, 'worlds', 'empty.world'))

    # # Navigation arguments
    # map_file = LaunchConfiguration('map_file', default=os.path.join(
    #     shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml'))
    # nav2_params_file = LaunchConfiguration(
    #     'nav2_params_file', default=nav2_params_file_path)
    # nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(
    #     shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))

    # # Map package arguments
    # map_env_params_file = LaunchConfiguration(
    #     'map_env_params_file', default=map_env_params_file_path)

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
            'use_rviz',
            default_value=use_rviz,
            choices=['true', 'false'],
            description='Flag to enable rviz visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='World used in the gazebo simulation'
        ),
        DeclareLaunchArgument(
            'gazebo_world_file',
            default_value=gazebo_world_file,
            description='World used in the gazebo simulation'
        ),

        # # Navigation arguments
        # DeclareLaunchArgument(
        #     'map_file',
        #     default_value=map_file,
        #     description='Full path to map file to load'
        # ),
        # DeclareLaunchArgument(
        #     'nav2_params_file',
        #     default_value=nav2_params_file_path,
        #     description='Full path to the nav2 params file to use'
        # ),
        # DeclareLaunchArgument(
        #     'nav2_rviz_config_file',
        #     default_value=nav2_rviz_config_file,
        #     description='Full path to the nav2 rviz config file to use'
        # ),

        # # Map package arguments
        # DeclareLaunchArgument(
        #     'map_env_params_file',
        #     default_value=map_env_params_file_path,
        #     description='Full path to the map_pkg params file to use'
        # ),
    ]

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(limo_gaze_pkg, 'launch'),
            '/robot.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'spawn_shlefino': 'false',
            'use_gui': use_gui,
            'use_rviz': use_rviz,
            'gazebo_world_file': gazebo_world_file
        }.items()
    )

    # # Rewrite the map file parameter substituting n_victims with 0
    # map_env_conf_params = RewrittenYaml(
    #     source_file=map_env_params_file,
    #     param_rewrites={'n_victims': '0'},
    #     convert_types=True
    # )

    # map_pkg_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(map_env_pkg, 'launch'),
    #         '/map_env.launch.py']
    #     ),
    #     launch_arguments={
    #         'map_env_params_file': map_env_conf_params,
    #     }.items()
    # )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
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

    # def launch_navigation(context):
    #     nodes = []
    #     for robot_id in range(int(context.launch_configurations['n_robot'])):

    #         robot_name = PythonExpression(
    #             ["'", "shelfino", str(robot_id), "'"])

    #         model_output = PythonExpression(["'", os.path.join(
    #             limo_desc_pkg, 'models', 'shelfino'), str(robot_id), ".sdf'"])
    #         sapf_node = GroupAction([
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource([
    #                     os.path.join(shelfino_nav2_pkg, 'launch'),
    #                     '/shelfino_nav.launch.py']
    #                 ),
    #                 launch_arguments={
    #                     'use_sim_time': use_sim_time,
    #                     'robot_id': str(robot_id),
    #                     'map_file': map_file,
    #                     'nav2_params_file': nav2_params_file,
    #                     'rviz_config_file': nav2_rviz_config_file,
    #                     'initial_x': str(robot_id*2.0-2.0),
    #                     'initial_y': str(robot_id*2.0-2.0),
    #                     'headless': 'true',
    #                 }.items()
    #             ),
    #         ])

    #         nodes.append(sapf_node)

    #     return nodes

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    # ld.add_action(OpaqueFunction(function=get_map_name))
    ld.add_action(OpaqueFunction(function=print_env))
    # ld.add_action(OpaqueFunction(function=evaluate_rviz))

    # create_map_node = Node(
    #     package='map_pkg',
    #     executable='create_map_pgm.py',
    #     name='create_map_pgm',
    #     output='screen',
    #     parameters=[map_env_conf_params]
    # )

    ld.add_action(gazebo_launch)
    # ld.add_action(map_pkg_node)

    # ld.add_action(TimerAction(
    #     period='2',
    #     actions=[create_map_node]
    # ))

    ld.add_action(TimerAction(
        period='5',
        actions=[OpaqueFunction(function=launch_rsp)]
    ))
    ld.add_action(TimerAction(
        period='10',
        actions=[OpaqueFunction(function=spawn_robot)]
    ))

    ld.add_action(TimerAction(
        period='15',
        actions=[rviz2_node]
    ))

    # ld.add_action(TimerAction(
    #     period='20',
    #     actions=[OpaqueFunction(function=launch_navigation)]
    # ))

    return ld