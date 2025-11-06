from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_offboard',
            executable='swarm_lawnmower_offboard.py',
            name='swarm_lawnmower',
            output='screen',
            emulate_tty=True
        )
    ])
