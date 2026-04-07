import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_my_robot = FindPackageShare('my_robot')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # 让 Gazebo 使用 ROS 2 时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='false')

    # 解析 URDF 文件
    xacro_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'diffbot.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),

        # 1. 启动 Gazebo 服务端 (包含 ROS 2 插件工厂，这是关键！)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
            )
        ),

        # 2. 启动 Gazebo 客户端 (显示画面)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
            ),
            condition=IfCondition(gui)
        ),

        # 3. 启动 Robot State Publisher (发布 TF 树)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_my_robot',
            remappings=[
                ('joint_states', 'wheel_joint_states'),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }],
            output='screen'
        ),

        # 4. 把车 Spawn 到 Gazebo 里
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_diffbot',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'diffbot',
                '-z', '0.1'
            ],
            output='screen'
        ),
    ])
