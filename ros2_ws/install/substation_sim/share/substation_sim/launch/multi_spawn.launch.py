import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
import xacro

def generate_launch_description():
    pkg_substation_sim = get_package_share_directory('substation_sim')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    
    # Robot URDF
    xacro_file = os.path.join(pkg_substation_sim, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    
    def spawn_robot(name, x, y):
        return GroupAction([
            PushRosNamespace(name),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'frame_prefix': f'{name}/'
                }]
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', name, '-x', str(x), '-y', str(y), '-z', '0.5'],
                output='screen'
            )
        ])
    
    # Spawn 3 robots
    ugv1 = spawn_robot('ugv1', -20, -20)
    ugv2 = spawn_robot('ugv2', -15, -20)
    ugv3 = spawn_robot('ugv3', -10, -20)
    
    return LaunchDescription([
        gazebo,
        ugv1,
        ugv2,
        ugv3
    ])
