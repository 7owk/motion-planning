#!/usr/bin/env python3
"""
ROS 2 Humble launch file for Btraj.

Brings up:
  * nav2_map_server  +  nav2_lifecycle_manager (configures + activates map_server)
  * tf_broadcaster   (static map -> odom)
  * astar            (A* path search, listens to /map, /initialpose, /goal_pose)
  * b_traj           (Bezier corridor + QP, listens to /path_to_btraj)
  * rviz2            (visualization)

Usage:
    ros2 launch btraj test.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('btraj')
    map_yaml  = os.path.join(pkg_share, 'maps', 'map.yaml')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'rviz.rviz')

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=map_yaml,
        description='Full path to the map yaml file to load'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch rviz2'
    )

    # ---- nav2 map_server (replaces ROS1 map_server) ----
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map'),
            'frame_id': 'map',
            'topic_name': 'map',
        }],
    )

    # nav2_map_server is a lifecycle node, so it needs configuring + activating.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ---- static tf map -> odom (10, 10, 0) ----
    tf_br = Node(
        package='btraj',
        executable='tf_br',
        name='tf_broadcaster',
        output='screen',
    )

    # ---- A* search ----
    astar = Node(
        package='btraj',
        executable='astar',
        name='astar',
        output='screen',
    )

    # ---- Bezier trajectory generator ----
    b_traj = Node(
        package='btraj',
        executable='b_traj',
        name='b_traj',
        output='screen',
        parameters=[{
            'max_inflate_iter': 2000,
            # ROS2 nested params use '.' separator
            'B_traj.order':  6,
            'B_traj.vx_max':  10.0,
            'B_traj.vx_min': -10.0,
            'B_traj.vy_max':  10.0,
            'B_traj.vy_min': -10.0,
            'B_traj.ax_max':  20.0,
            'B_traj.ax_min': -20.0,
            'B_traj.ay_max':  20.0,
            'B_traj.ay_min': -20.0,
        }],
    )

    # ---- rviz2 ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        map_yaml_arg,
        use_rviz_arg,
        map_server,
        lifecycle_manager,
        tf_br,
        astar,
        b_traj,
        rviz,
    ])
