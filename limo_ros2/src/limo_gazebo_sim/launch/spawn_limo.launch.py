from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'limo_description'
    xacro_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        'limo_base.urdf'
    ])

    gazebo_ros = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_ros),
            launch_arguments={'verbose': 'true'}.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'limo',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.0'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'rviz',
                'display_config.rviz'
            ])],
            output='screen'
        )
    ])
