from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    param_file = os.path.join(
        get_package_share_directory('sbim_launch'),
        'config',
        'monstar_towne_levine.yaml'
    )

    return LaunchDescription([
        SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp"),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging verbosity level'
        ),

        # Scan aggregator
        Node(
            package='scan_aggregator',
            executable='scan_aggregator',
            name='scan_aggregator',
            output='screen',
            remappings=[
                ('/scan', '/rig/monstar/points'),
                ('/odometry', '/rig/vio/odom')
            ],
            parameters=[param_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('structural_compass'), '/launch/point_cloud_compass.launch.py'])
        ),
        Node(
            package='tf_publishers',
            executable='compass_to_tf',
            name='compass_to_tf',
            output='screen',
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([FindPackageShare('scene_parsing'), '/launch/scene_parsing.launch.py'])
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([FindPackageShare('planar_slam'), '/launch/planar_slam.launch.py'])
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([FindPackageShare('floorplan_estimation'), '/launch/floorplan_estimation.launch.py'])
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([FindPackageShare('sbim_launch'), '/launch/visualization.launch.py'])
        # ),
    ])