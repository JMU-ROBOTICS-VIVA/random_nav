#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    tb_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')

    default_world_path = os.path.join(get_package_share_directory('random_nav'), 'worlds', 'room.world')
    default_map_path = os.path.join(get_package_share_directory('random_nav'), 'maps', 'room.yaml')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map_path),
        # START SIMULATOR
        #ExecuteProcess(
        #    cmd=['gazebo', '--verbose', LaunchConfiguration('world'), '-s', 'libgazebo_ros_init.so'],
        #    output='screen'),
        #   
        #ExecuteProcess(
        #    cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        #    output='screen'),
        #
        #Node(
        #    package="jmu_turtlebot3_bringup",
        #    executable="tb_fixer",
        #    name="tb_fixer",
        #    output="screen",
        #),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': default_world_path}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # START NAV SYSTEM
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation2.launch.py']),
            launch_arguments=[
                ('map', LaunchConfiguration('map')),
                ('use_sim_time', use_sim_time)
            ],
        ),
        
    ])

if __name__ == "__main__":
    generate_launch_description()
