from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_pkg = FindPackageShare('limo_description')
    sim_pkg = FindPackageShare('limo_gazebo_sim')
    gazebo_pkg = FindPackageShare('gazebo_ros')

    urdf_file = PathJoinSubstitution([description_pkg, 'urdf', 'limo_base.urdf'])
    config_file = ParameterFile(
        PathJoinSubstitution([sim_pkg, 'config', 'arm_trajectory_controller.yaml']),
        allow_substs=True
    )

    robot_desc = {'robot_description': Command(['xacro ', urdf_file])}


    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_pkg, 'launch', 'gazebo.launch.py'])
            )
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_desc, {'use_sim_time': True}],
            output='screen'
        ),

        # Spawn entity into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'limo', '-topic', 'robot_description'],
            output='screen'
        ),

        # ROS 2 control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[robot_desc, {'use_sim_time': True}, config_file],
            remappings=[('~/robot_description', '/robot_description')],
            output='screen'
        ),

        # Spawner nodes
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_trajectory_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_action_controller'],
            output='screen'
        ),
    ])
