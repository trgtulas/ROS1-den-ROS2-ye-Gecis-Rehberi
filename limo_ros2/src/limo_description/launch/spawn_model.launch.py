from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'limo_description'
    urdf_file = 'limo_base.urdf'
    rviz_config = 'display_config.rviz'

    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file)
    rviz_path = os.path.join(get_package_share_directory(package_name), 'rviz', rviz_config)

    with open(urdf_path, 'r') as file:
        robot_description_content = file.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        )
    ])
