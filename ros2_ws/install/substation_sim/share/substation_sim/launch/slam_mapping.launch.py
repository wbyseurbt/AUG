from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    start_teleop = LaunchConfiguration('start_teleop')
    slam_method = LaunchConfiguration('slam_method')
    rviz = LaunchConfiguration('rviz')

    cartographer_config_dir = PathJoinSubstitution([FindPackageShare('substation_sim'), 'config'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare('substation_sim'), 'rviz', 'slam_mapping.rviz'])

    sim_and_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_robot'), 'launch', 'substation_combo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'start_teleop': start_teleop,
        }.items(),
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_substation.lua',
        ],
        remappings=[('scan', '/scan')],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'cartographer'"])),
    )

    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'cartographer'"])),
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_laser_range': 30.0,
            'minimum_time_interval': 0.2,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': False,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': 'scan',
            'mode': 'mapping'
        }],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'slam_toolbox'"])),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_teleop', default_value='false'),
        DeclareLaunchArgument('slam_method', default_value='cartographer'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim_and_robot,
        cartographer_node,
        cartographer_grid_node,
        slam_toolbox_node,
        rviz_node,
    ])
