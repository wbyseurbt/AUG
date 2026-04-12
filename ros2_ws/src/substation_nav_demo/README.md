# substation_nav_demo

用于演示“单车自主导航 + 动态避障 + RViz 可视化”的独立 ROS2 包。

## 功能

- 启动 Gazebo 变电站场景与小车
- 加载已保存地图并启动 Nav2
- 使用窄路参数（local/global costmap）
- 自动发送导航目标（`NavigateToPose`）
- 运行中延时生成临时障碍物，触发局部重规划

## 依赖（Ubuntu + ROS2 Humble）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs
```

## 编译

```bash
cd /root/dev_ws/AUG/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select substation_nav_demo
source install/setup.bash
```

## 一键启动演示

```bash
ros2 launch substation_nav_demo autonomy_obstacle_demo.launch.py
```

说明：launch 已默认将 FastDDS 传输切换为 `UDPv4`，用于规避部分环境下大量 `RTPS_TRANSPORT_SHM` 端口锁告警。

如果你要加载 SLAM 保存的全局地图（推荐）：

```bash
ros2 launch substation_nav_demo autonomy_obstacle_demo.launch.py \
  map:=/root/dev_ws/AUG/ros2_ws/src/substation_sim/maps/substation_global.yaml
```

## 运行效果观察

- RViz 中可看到：
  - 静态地图、激光点云
  - `/goal_markers` 目标箭头与文字（Goal 1 / Goal 2）
  - 全局路径 / 局部轨迹
  - 机器人位姿变化
- Gazebo 中可看到：
  - 绿色起点标记（`start_marker`）
  - 红色目标点标记（`goal1_marker`、`goal2_marker`）
- 约 `obstacle_delay_sec` 秒后，会在默认路径前方生成一个红色方块障碍物，机器人应触发局部路径重规划并绕行。

重要：如果 AMCL 提示 `Please set the initial pose`，请在 RViz 点击 `2D Pose Estimate` 在机器人初始位置点一下方向，随后会建立 `map->odom`，自动目标才会被接受。
默认 RViz 配置已启用目标可视化：`substation_nav_demo/config/nav_demo.rviz`。

## 常用调参

```bash
# 修改地图
ros2 launch substation_nav_demo autonomy_obstacle_demo.launch.py \
  map:=/root/dev_ws/AUG/ros2_ws/src/substation_sim/maps/substation_global.yaml

# 调整障碍物位置
ros2 launch substation_nav_demo autonomy_obstacle_demo.launch.py \
  obstacle_ahead_distance:=1.4 obstacle_lateral_offset:=0.1 obstacle_delay_sec:=15.0

# 调整目标点
ros2 launch substation_nav_demo autonomy_obstacle_demo.launch.py \
  goal_1_x:=5.0 goal_1_y:=1.0 goal_2_x:=0.0 goal_2_y:=0.0
```

说明：默认障碍物会在生成时刻根据 `/odom` 中车体朝向，动态放在车头前方（`obstacle_ahead_distance`）并可带侧向偏置（`obstacle_lateral_offset`）。
说明：默认 `goal_frame:=map`（Nav2 全局导航更稳定）；若你明确要用里程计坐标，可手动改为 `goal_frame:=odom`。

## 代码说明

- 仅保留当前演示必需代码（单一 launch 入口 + 三个脚本节点 + 三类模型资源）。
- 删除了无效参数链路（如未被实际使用的 `target_model` 参数），避免后续维护混淆。
