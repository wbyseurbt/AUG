from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
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
    obstacle_min_ahead_distance = LaunchConfiguration('obstacle_min_ahead_distance')
    obstacle_lateral_offset = LaunchConfiguration('obstacle_lateral_offset')
    obstacle_random_lateral_range = LaunchConfiguration('obstacle_random_lateral_range')
    obstacle_z = LaunchConfiguration('obstacle_z')
    obstacle_start_delay = LaunchConfiguration('obstacle_start_delay_sec')
    obstacle_period = LaunchConfiguration('obstacle_period_sec')
    enable_random_obstacle = LaunchConfiguration('enable_random_obstacle')

    facility_csv = LaunchConfiguration('facility_csv')
    route_start_x = LaunchConfiguration('route_start_x')
    route_start_y = LaunchConfiguration('route_start_y')
    route_end_x = LaunchConfiguration('route_end_x')
    route_end_y = LaunchConfiguration('route_end_y')
    goal_delay = LaunchConfiguration('goal_start_delay_sec')
    goal_frame = LaunchConfiguration('goal_frame')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    substation_sim_share = FindPackageShare('substation_sim')
    my_robot_share = FindPackageShare('my_robot')
    demo_share = FindPackageShare('substation_nav_demo')
    obstacle_model_file = PathJoinSubstitution([demo_share, 'models', 'temporary_box', 'model.sdf'])
    ekf_config_file = PathJoinSubstitution([demo_share, 'config', 'ekf_2d.yaml'])

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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav_demo',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    obstacle_spawner = Node(
        package='substation_nav_demo',
        executable='spawn_obstacle_ahead.py',
        name='spawn_obstacle_ahead',
        condition=IfCondition(enable_random_obstacle),
        parameters=[{
            'entity_prefix': 'temporary_box',
            'model_file': obstacle_model_file,
            'ahead_distance': obstacle_ahead_distance,
                    'min_ahead_distance': obstacle_min_ahead_distance,
            'lateral_offset': obstacle_lateral_offset,
            'random_lateral_range': obstacle_random_lateral_range,
            'z': obstacle_z,
            'start_delay_sec': obstacle_start_delay,
            'spawn_period_sec': obstacle_period,
            'wait_timeout_sec': 30.0,
        }],
        output='screen',
    )

    start_marker_spawner = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'start_marker',
                    '-file', PathJoinSubstitution([demo_share, 'models', 'start_marker', 'model.sdf']),
                    '-x', route_start_x,
                    '-y', route_start_y,
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
                    '-entity', 'goal2_marker',
                    '-file', PathJoinSubstitution([demo_share, 'models', 'goal_marker', 'model.sdf']),
                    '-x', route_end_x,
                    '-y', route_end_y,
                    '-z', '0.02',
                    '-Y', '0.0',
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
                'facility_csv': facility_csv,
                'route_start_x': route_start_x,
                'route_start_y': route_start_y,
                'route_end_x': route_end_x,
                'route_end_y': route_end_y,
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
                'facility_csv': facility_csv,
                'route_start_x': route_start_x,
                'route_start_y': route_start_y,
                'route_end_x': route_end_x,
                'route_end_y': route_end_y,
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
        DeclareLaunchArgument('obstacle_ahead_distance', default_value='1.6'),
        DeclareLaunchArgument('obstacle_min_ahead_distance', default_value='1.6'),
        DeclareLaunchArgument('obstacle_lateral_offset', default_value='0.0'),
        DeclareLaunchArgument('obstacle_random_lateral_range', default_value='0.20'),
        DeclareLaunchArgument('obstacle_z', default_value='0.1'),
        DeclareLaunchArgument('obstacle_start_delay_sec', default_value='30.0'),
        DeclareLaunchArgument('obstacle_period_sec', default_value='60.0'),
        DeclareLaunchArgument('enable_random_obstacle', default_value='true'),
        DeclareLaunchArgument('goal_start_delay_sec', default_value='12.0'),
        DeclareLaunchArgument(
            'facility_csv',
            default_value=PathJoinSubstitution([substation_sim_share, 'config', 'obstacles.csv'])
        ),
        DeclareLaunchArgument('route_start_x', default_value='-35.0'),
        DeclareLaunchArgument('route_start_y', default_value='-35.0'),
        DeclareLaunchArgument('route_end_x', default_value='45.0'),
        DeclareLaunchArgument('route_end_y', default_value='40.0'),
        DeclareLaunchArgument('goal_frame', default_value='map'),
        DeclareLaunchArgument('spawn_x', default_value='-35.0'),
        DeclareLaunchArgument('spawn_y', default_value='-35.0'),
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
        ekf_node,
        nav2_launch,
        rviz_node,
        goal_marker_pub,
        goal_sender,
        start_marker_spawner,
        goal1_marker_spawner,
        obstacle_spawner,
    ])
