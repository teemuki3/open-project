from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
		    package='merge_map',
		    executable='merge_map',
		    output='screen',
		    parameters=[{'use_sim_time': True}],
		    remappings=[
			("/map1", "/yourMapName1"),
			("/map2", "/yourMapName2"),
		    ],
		),
	])
