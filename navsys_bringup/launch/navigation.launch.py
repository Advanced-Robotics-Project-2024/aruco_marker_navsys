# SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import GroupAction

def generate_launch_description():
    pkg_dir = get_package_share_directory('navsys_bringup')
    params_file = os.path.join(pkg_dir, 'config', 'param_sim.yaml')

    load_nodes = GroupAction(
            actions=[
                Node(
                    package='aruco_marker_detector_py', 
                    executable='aruco_marker_info_pub',
                    name='aruco_marker_info_pub'), 
                Node(
                    package='aruco_marker_navigation', 
                    executable='rotate_action', 
                    name='rotate_action',
                    parameters=[{'max_angular_vel': 0.3}]),
                Node(
                    package='aruco_marker_navigation', 
                    executable='adjust_direction', 
                    name='adjust_direction', 
                    parameters=[{'max_angular_vel': 0.3}]),
                Node(
                    package='aruco_marker_navigation', 
                    executable='adjust_position', 
                    name='adjust_position', 
                    parameters=[{'max_linear_vel': 0.22}]),
                Node(
                    package='aruco_marker_navigation', 
                    executable='approach_marker', 
                    name='approach_marker', 
                    parameters=[{'max_linear_vel': 0.2}])
                ]
    )

    ld = LaunchDescription()

    ld.add_action(load_nodes)
    
    return ld

