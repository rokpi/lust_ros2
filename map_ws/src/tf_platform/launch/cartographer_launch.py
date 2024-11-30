from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cartographer_config_dir = os.path.join(
        get_package_share_directory('tf_platform'),
        'config')
    configuration_basename = 'cartographer_config.lua'
    configuration_directory = cartographer_config_dir

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('/scan', '/scan')
            ]),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            remappings=[
                ('/submap_list', '/submap_list'),
                ('/map', '/map')
            ]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        )
    ])
