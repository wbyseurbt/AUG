from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _build_multi_robot_actions(context):
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution([FindPackageShare('my_robot'), 'urdf', 'diffbot.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])

    spawn_points = [(-35.0, -35.0), (-28.0, -35.0)]
    actions = []

    for i in range(2):
        robot_name = f'ugv{i + 1}'
        spawn_x, spawn_y = spawn_points[i]

        actions.append(
            GroupAction([
                PushRosNamespace(robot_name),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': robot_description,
                    }],
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                    output='screen',
                ),
            ])
        )

        # Delay spawn slightly so Gazebo services are ready and multi-robot spawn is stable.
        actions.append(
            TimerAction(
                period=3.0 + i * 1.5,
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name=f'spawn_{robot_name}',
                        arguments=[
                            '-topic', f'/{robot_name}/robot_description',
                            '-entity', robot_name,
                            '-robot_namespace', f'/{robot_name}',
                            '-x', str(spawn_x),
                            '-y', str(spawn_y),
                            '-z', str(spawn_z),
                            '-timeout', '120.0',
                        ],
                        output='screen',
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    substation_sim_share = FindPackageShare('substation_sim')
    demo_share = FindPackageShare('substation_nav_demo')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    waypoints_file = LaunchConfiguration('waypoints_file')
    dwell_sec = LaunchConfiguration('dwell_sec')
    capture_dir = LaunchConfiguration('capture_dir')
    gazebo_master_uri = LaunchConfiguration('gazebo_master_uri')

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_master_uri', default_value='http://127.0.0.1:11346'),
        SetEnvironmentVariable('HOME', '/tmp'),
        SetEnvironmentVariable('ROS_LOG_DIR', '/tmp/ros_log'),
        SetEnvironmentVariable('GAZEBO_HOME', '/tmp/gazebo'),
        SetEnvironmentVariable('GAZEBO_LOG_PATH', '/tmp/gazebo_log'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', gazebo_master_uri),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            PathJoinSubstitution([substation_sim_share, 'models'])
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([substation_sim_share, 'worlds', 'substation.world'])
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([demo_share, 'config', 'multi_ugv_waypoints.yaml'])
        ),
        DeclareLaunchArgument('dwell_sec', default_value='4.0'),
        DeclareLaunchArgument('capture_dir', default_value='/home/zzz/AUG/ros2_ws/screenshots'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={
                'world': world,
                'gui': gui,
                'verbose': 'true',
            }.items(),
        ),
        OpaqueFunction(function=_build_multi_robot_actions),
        Node(
            package='substation_nav_demo',
            executable='multi_ugv_patrol_controller.py',
            name='multi_ugv_patrol_controller',
            parameters=[{
                'use_sim_time': use_sim_time,
                'waypoints_file': waypoints_file,
                'dwell_sec': dwell_sec,
                'capture_dir': capture_dir,
            }],
            output='screen',
        ),
    ])
