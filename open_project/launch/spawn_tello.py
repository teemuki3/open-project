from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    ns1 = 'tello_drone'
    
    return LaunchDescription([
        
        # Spawn tello.urdf in front of turtlebot
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '1', '0', '1', '1.57079632679']),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),
             
        # Joystick controller, generates /namespace/cmd_vel messages for tello control client.
        Node(package='tello_driver', executable='tello_joy_main', output='screen',
             namespace=ns1),
             
        # Liftoff client node to send the takeoff command to the drone
        Node(package='tello_gazebo', executable='takeoff_client_node.py', output='screen',),

    ])