import os
import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_param_node = Node(
        package='robot_param',
        executable='robot_param', 
        output='screen',
        emulate_tty=True,
        parameters=["/home/roy/dev_ws/src/robot_desktop/robot_param/config/robot_param.yaml"]
    )
        
    return LaunchDescription([
        robot_param_node
    ])
