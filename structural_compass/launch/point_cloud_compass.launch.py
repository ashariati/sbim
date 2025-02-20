import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='structural_compass',
            executable='point_cloud_compass_node',
            name='point_cloud_compass_node',
            remappings=[
                ('/pose', '/keyframe'),
                ('/scan', '/aggregate_scan')
            ]
        ),
    ])
