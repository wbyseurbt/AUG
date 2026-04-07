import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    pkg_substation_sim = get_package_share_directory('substation_sim')
    world_path = os.path.join(pkg_substation_sim, 'worlds', 'road_collision_test.world')
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_substation_sim, 'models')
    gz_bin = shutil.which("gazebo")
    if gz_bin:
        return LaunchDescription([
            LogInfo(msg=f"启动道路碰撞测试: {world_path}"),
            LogInfo(msg="观察红球是否落在灰色道路上并静止；若是，说明道路碰撞层生效。"),
            ExecuteProcess(
                cmd=[gz_bin, "--verbose", world_path],
                output="screen",
                additional_env={
                    "GAZEBO_MODEL_DATABASE_URI": "",
                },
            ),
        ])
    return LaunchDescription([
        LogInfo(msg="当前环境未安装 gazebo，无法执行碰撞测试。"),
        LogInfo(msg=f"测试 world 文件: {world_path}"),
    ])
