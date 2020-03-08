from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roomba_600_driver',
            node_executable='tf_broadcast',
            output='screen'),
            # parameters=[config]),
    ])
