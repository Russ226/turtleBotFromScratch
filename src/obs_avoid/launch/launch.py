import os
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
     return LaunchDescription([
        Node(
            package='obs_avoid',
            executable='avoid_calc',
            name='avoid_calc',
        )
    ])