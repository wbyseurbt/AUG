from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_robot = FindPackageShare("my_robot")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    sim_launch = PathJoinSubstitution([pkg_my_robot, "launch", "sim.launch.py"])
    rviz_config = PathJoinSubstitution([pkg_my_robot, "rviz", "diffbot.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={"use_sim_time": use_sim_time, "gui": "true"}.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="screen",
                    )
                ],
            ),
        ]
    )
