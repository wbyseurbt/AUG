from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    rviz_config = LaunchConfiguration('rviz_config')
    autostart = LaunchConfiguration('autostart')

    obstacle_ahead_distance = LaunchConfiguration('obstacle_ahead_distance')
    obstacle_lateral_offset = LaunchConfiguration('obstacle_lateral_offset')
    obstacle_z = LaunchConfiguration('obstacle_z')
    obstacle_delay = LaunchConfiguration('obstacle_delay_sec')

    goal1_x = LaunchConfiguration('goal_1_x')
    goal1_y = LaunchConfiguration('goal_1_y')
    goal1_yaw = LaunchConfiguration('goal_1_yaw')
    goal2_x = LaunchConfiguration('goal_2_x')
    goal2_y = LaunchConfiguration('goal_2_y')
    goal2_yaw = LaunchConfiguration('goal_2_yaw')
    goal_delay = LaunchConfiguration('goal_start_delay_sec')
    goal_frame = LaunchConfiguration('goal_frame')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    substation_sim_share = FindPackageShare('substation_sim')
    my_robot_share = FindPackageShare('my_robot')
    demo_share = FindPackageShare('substation_nav_demo')
    obstacle_model_file = PathJoinSubstitution([demo_share, 'models', 'temporary_box', 'model.sdf'])

    combo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([my_robot_share, 'launch', 'substation_combo.launch.py'])
        ),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_sim_time,
            'start_teleop': 'false',
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'entity_name': 'diffbot',
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': autostart,
            'slam': 'False',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav_demo',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    obstacle_spawner = TimerAction(
        period=obstacle_delay,
        actions=[
            Node(
                package='substation_nav_demo',
                executable='spawn_obstacle_ahead.py',
                name='spawn_obstacle_ahead',
                parameters=[{
                    'entity_name': 'temporary_box',
                    'model_file': obstacle_model_file,
                    'ahead_distance': obstacle_ahead_distance,
                    'lateral_offset': obstacle_lateral_offset,
                    'z': obstacle_z,
                    'wait_timeout_sec': 30.0,
                }],
                output='screen',
            )
        ],
    )

    start_marker_spawner = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'start_marker',
                    '-file', PathJoinSubstitution([demo_share, 'models', 'start_marker', 'model.sdf']),
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.02',
                    '-Y', '0.0',
                ],
                output='screen',
            )
        ],
    )

    goal1_marker_spawner = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'goal1_marker',
                    '-file', PathJoinSubstitution([demo_share, 'models', 'goal_marker', 'model.sdf']),
                    '-x', goal1_x,
                    '-y', goal1_y,
                    '-z', '0.02',
                    '-Y', goal1_yaw,
                ],
                output='screen',
            )
        ],
    )

    goal2_marker_spawner = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'goal2_marker',
                    '-file', PathJoinSubstitution([demo_share, 'models', 'goal_marker', 'model.sdf']),
                    '-x', goal2_x,
                    '-y', goal2_y,
                    '-z', '0.02',
                    '-Y', goal2_yaw,
                ],
                output='screen',
            )
        ],
    )

    goal_sender = Node(
        package='substation_nav_demo',
        executable='auto_goal_sender.py',
        name='auto_goal_sender',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'start_delay_sec': goal_delay,
                'frame_id': goal_frame,
                'goal_1_x': goal1_x,
                'goal_1_y': goal1_y,
                'goal_1_yaw': goal1_yaw,
                'goal_2_x': goal2_x,
                'goal_2_y': goal2_y,
                'goal_2_yaw': goal2_yaw,
                'enable_second_goal': True,
            },
        ],
        output='screen',
    )

    goal_marker_pub = Node(
        package='substation_nav_demo',
        executable='goal_marker_publisher.py',
        name='goal_marker_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'frame_id': goal_frame,
                'goal_1_x': goal1_x,
                'goal_1_y': goal1_y,
                'goal_1_yaw': goal1_yaw,
                'goal_2_x': goal2_x,
                'goal_2_y': goal2_y,
                'goal_2_yaw': goal2_yaw,
                'enable_second_goal': True,
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([substation_sim_share, 'maps', 'substation_edited.yaml'])
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([demo_share, 'config', 'nav2_narrow_road_params.yaml'])
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([demo_share, 'config', 'nav_demo.rviz'])
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('obstacle_ahead_distance', default_value='1.2'),
        DeclareLaunchArgument('obstacle_lateral_offset', default_value='0.0'),
        DeclareLaunchArgument('obstacle_z', default_value='0.1'),
        DeclareLaunchArgument('obstacle_delay_sec', default_value='18.0'),
        DeclareLaunchArgument('goal_start_delay_sec', default_value='12.0'),
        DeclareLaunchArgument('goal_1_x', default_value='30.0'),
        DeclareLaunchArgument('goal_1_y', default_value='0.0'),
        DeclareLaunchArgument('goal_1_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_2_x', default_value='0.0'),
        DeclareLaunchArgument('goal_2_y', default_value='0.0'),
        DeclareLaunchArgument('goal_2_yaw', default_value='3.14'),
        DeclareLaunchArgument('goal_frame', default_value='map'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/DeleteEntity', "{name: 'temporary_box'}"],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/DeleteEntity', "{name: 'start_marker'}"],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/DeleteEntity', "{name: 'goal1_marker'}"],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/DeleteEntity', "{name: 'goal2_marker'}"],
            output='screen',
        ),
        combo_launch,
        nav2_launch,
        rviz_node,
        goal_marker_pub,
        goal_sender,
        start_marker_spawner,
        goal1_marker_spawner,
        goal2_marker_spawner,
        obstacle_spawner,
    ])
