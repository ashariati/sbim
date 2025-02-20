import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():


    # param_file = os.path.join(
    #     get_package_share_directory('sbim_launch'),
    #     'config',
    #     'monstar_towne_levine.yaml'
    # )
    # print(param_file)

    return LaunchDescription([
        # Declare the launch argument for logging verbosity
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging verbosity level'
        ),

        # Node configuration
        Node(
            package='scan_aggregator',
            executable='scan_aggregator',
            name='scan_aggregator',
            output='screen',
            remappings=[
                ('/scan', '/rig/monstar/points'),
                ('/odometry', '/rig/vio/odom')
            ],
            # parameters=[param_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

    ])

if __name__ == '__main__':
    generate_launch_description()