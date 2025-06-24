import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    pkg = 'limo_description'
    xacro = PathJoinSubstitution([
        FindPackageShare(pkg), 'urdf', 'limo_only.urdf'
    ])
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')

    robot_desc_cmd = Command(['xacro ', xacro])
    gazebo_launch = PathJoinSubstitution([gazebo_pkg, 'launch', 'gazebo.launch.py'])

    config_file = ParameterFile(
        os.path.join(
            get_package_share_directory('limo_gazebo_sim'), 'config', 'limo_controllers.yaml'
        ),
        allow_substs=True
    )

    world_path = os.path.join(
        FindPackageShare('limo_gazebo_sim').find('limo_gazebo_sim'), 
        'worlds',
        'simple.world'
    )

    # world = LaunchConfiguration('world')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')


    # declare_world_cmd = DeclareLaunchArgument(
    #     name='world',
    #     default_value=world_path,
    #     description='Full path to the world model file to load')
    
    declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

    declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),
        declare_simulator_cmd,
        declare_use_simulator_cmd,
        # Launch Gazebo
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('gazebo_ros'),
        #             'launch',
        #             'gazebo.launch.py'
        #         ])
        #     ])
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')),
            condition=IfCondition(use_simulator),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'limo', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # State publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc_cmd, 'use_sim_time': True}],
            output='screen'
        ),
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen'),


        # Spawners for controllers
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=[
        #         'diff_drive_controller', 'joint_state_broadcaster',
        #         '--controller-manager', '/controller_manager', "--param-file", 
        #         '--controller-manager-timeout', '50',
        #     ],
        #     parameters=[{'use_sim_time': True}],
        #     output='screen'
        # ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare(pkg), 'rviz', 'display_config.rviz'])],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
