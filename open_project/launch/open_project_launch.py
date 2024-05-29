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
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    maze_world = os.path.join(get_package_share_directory('open_project'), 'worlds', 'maze.world')
    tello_urdf = os.path.join(get_package_share_directory('open_project'), 'urdf', 'tello_seeker.urdf')
    turtle_urdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf')
    turtle_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    ld = LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')), launch_arguments={'world': maze_world}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')), launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': get_package_share_directory('open_project')}.items()),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),

        # Robot 1
        Node(package='open_project', executable='drone', namespace='robot1', output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='robot1', output='screen', parameters=[{'use_sim_time': use_sim_time}], arguments=[tello_urdf]),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'tello', '-file', tello_urdf, '-x', '1.0', '-y', '0.0', '-z', '1.0'], output='screen'),

        # Robot 2
        #Node(package='open_project', executable='robot', namespace='robot2', output='screen', parameters=[{'use_sim_time': use_sim_time}]), # Tässä on jotain helloworld paskaa vaan
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='robot2', output='screen', parameters=[{'use_sim_time': use_sim_time}], arguments=[turtle_urdf]),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', TURTLEBOT3_MODEL, '-file', turtle_sdf, '-x', '0.0', '-y', '0.0', '-z', '0.0'], output='screen'),

        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', output='screen', parameters=[os.path.join(get_package_share_directory('open_project'), 'pointcloud_to_laserscan_params.yaml')], remappings=[('cloud_in', '/depth_camera/points'), ('scan', 'ultrascan')]),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', output='screen', parameters=[os.path.join(get_package_share_directory('open_project'), 'slam_params.yaml')])

    ])
    return ld
