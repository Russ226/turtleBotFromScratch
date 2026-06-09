import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    include_child_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.expanduser('~/fromScratchTurtleBot/src/ldlidar_stl_ros2/launch/ld19.launch.py'))
    )
    return LaunchDescription([
        include_child_launch,
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_node',
            output='screen',
            # Define arguments for UDP transport (e.g., port 8888)
            arguments=['serial', '--dev', '/dev/ttyACM0']
            # Define arguments for Serial transport (e.g., replace '/dev/ttyUSB0' with your port)
            # arguments=['serial', '--dev', '/dev/ttyUSB0']
        ),
        Node(
            package='obs_avoid',
            executable='avoid_calc',
            name='avoid_calc'
        )
    ])
