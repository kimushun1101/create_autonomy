from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

config = LaunchConfiguration(
         'params', default=[ThisLaunchFileDir(), '/default.yaml'])

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ca_driver', 
            node_executable='ca_driver',
            output='screen'),
            # parameters=[config]),
    ])
