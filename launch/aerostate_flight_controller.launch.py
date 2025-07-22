import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aerostat_control',
            executable='flight_controller',
            name='flight_controller',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('aerostat_control'),
                'config', 'params.yaml'
            )]
        )
    ])