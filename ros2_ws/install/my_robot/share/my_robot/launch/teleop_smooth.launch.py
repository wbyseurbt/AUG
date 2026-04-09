from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate = LaunchConfiguration("rate")
    timeout = LaunchConfiguration("timeout")
    speed = LaunchConfiguration("speed")
    turn = LaunchConfiguration("turn")
    start_teleop = LaunchConfiguration("start_teleop")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rate", default_value="100"),
            DeclareLaunchArgument("timeout", default_value="0.2"),
            DeclareLaunchArgument("speed", default_value="0.5"),
            DeclareLaunchArgument("turn", default_value="1.4"),
            DeclareLaunchArgument("start_teleop", default_value="false"),
            LogInfo(
                condition=UnlessCondition(start_teleop),
                msg="teleop_twist_keyboard 需要TTY。请在另一个终端手动运行: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop",
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                parameters=[{"speed": speed, "turn": turn}],
                remappings=[("cmd_vel", "cmd_vel_teleop")],
                output="screen",
                emulate_tty=True,
                condition=IfCondition(start_teleop),
            ),
            Node(
                package="my_robot",
                executable="cmd_vel_repeater.py",
                arguments=[
                    "--in",
                    "cmd_vel_teleop",
                    "--out",
                    "cmd_vel",
                    "--rate",
                    rate,
                    "--timeout",
                    timeout,
                ],
                output="screen",
            ),
        ]
    )
