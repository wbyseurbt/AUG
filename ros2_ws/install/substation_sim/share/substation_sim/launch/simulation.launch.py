import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo

def generate_launch_description():
    pkg_substation_sim = get_package_share_directory('substation_sim')
    world_file_name = 'substation.world'
    world_path = os.path.join(pkg_substation_sim, 'worlds', world_file_name)
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_substation_sim, 'models')
    gz_bin = shutil.which("gazebo")
    if gz_bin:
        return LaunchDescription([
            LogInfo(msg=f"启动 Gazebo 地图: {world_path}"),
            ExecuteProcess(
                cmd=[gz_bin, "--verbose", world_path],
                output="screen",
                additional_env={
                    "GAZEBO_MODEL_DATABASE_URI": "",
                },
            ),
        ])
    return LaunchDescription([
        LogInfo(msg="当前环境未安装 gazebo/gazebo_ros，已生成地图文件，请在安装 Gazebo 的机器运行。"),
        LogInfo(msg=f"地图文件: {world_path}"),
    ])
