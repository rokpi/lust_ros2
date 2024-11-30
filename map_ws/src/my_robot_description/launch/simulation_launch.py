import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')
    
    # Path to the Xacro file
    xacro_file = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf.xacro')
    localization_config = os.path.join(pkg_dir, 'config', 'my_robot_localization.yaml')

    # Process the Xacro file and get the URDF XML content
    urdf_content = xacro.process_file(xacro_file).toxml()

    # Write the processed URDF to a temporary file (optional but useful for debugging)
    urdf_temp_file = '/tmp/my_robot.urdf'
    with open(urdf_temp_file, 'w') as f:
        f.write(urdf_content)

    # Include the Gazebo launch file with necessary ROS 2 plugins
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        # Start Gazebo with ROS integration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),
        # Spawn the robot in Gazebo using the processed URDF file
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_temp_file],
            output='screen'
        ),
        # Start the robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_content}]
        ),
        # Start the EKF node for localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[localization_config],
        ),
    ])
