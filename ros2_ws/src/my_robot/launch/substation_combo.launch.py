import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_robot = FindPackageShare('my_robot')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_substation_sim = FindPackageShare('substation_sim')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_teleop = LaunchConfiguration('start_teleop')
    rate = LaunchConfiguration('rate')
    timeout = LaunchConfiguration('timeout')
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    entity_name = LaunchConfiguration('entity_name')

    world_path = PathJoinSubstitution([pkg_substation_sim, 'worlds', 'substation.world'])
    xacro_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'diffbot.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])
    model_path = PathJoinSubstitution([pkg_substation_sim, 'models'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_teleop', default_value='true'),
        DeclareLaunchArgument('rate', default_value='100'),
        DeclareLaunchArgument('timeout', default_value='0.2'),
        DeclareLaunchArgument('speed', default_value='0.5'),
        DeclareLaunchArgument('turn', default_value='1.4'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument('entity_name', default_value='diffbot'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={
                'world': world_path,
                'gui': gui,
                'verbose': 'true'
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_my_robot',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_diffbot',
            arguments=[
                '-topic', 'robot_description',
                '-entity', entity_name,
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z
            ],
            output='screen'
        ),
        LogInfo(
            condition=UnlessCondition(start_teleop),
            msg='teleop_twist_keyboard 需要TTY。请在另一个终端手动运行: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            parameters=[{'speed': speed, 'turn': turn}],
            remappings=[('cmd_vel', 'cmd_vel_teleop')],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(start_teleop),
        ),
        Node(
            package='my_robot',
            executable='cmd_vel_repeater.py',
            arguments=[
                '--in', 'cmd_vel_teleop',
                '--out', 'cmd_vel',
                '--rate', rate,
                '--timeout', timeout,
            ],
            output='screen',
        ),
    ])
