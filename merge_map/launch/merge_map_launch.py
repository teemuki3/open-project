# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='merge_map',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
            ("/map1", "/map"),
            ("/map2", "/map"),
            ],
        ),
    ])
