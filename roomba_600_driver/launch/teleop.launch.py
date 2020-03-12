from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

config = LaunchConfiguration(
        'params', default=[ThisLaunchFileDir(), '/teleop.yaml'])

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            node_executable='joy_node'),
        Node(
            package='roomba_600_driver',
            node_executable='teleop_node',
            output="screen",
            parameters=[config]),
    ])
