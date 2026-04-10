# 变电站全局建图（SLAM）使用说明

## 1. 环境依赖安装（首次执行）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-cartographer-ros \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools
```

安装完成后建议开一个新终端，避免旧环境变量干扰。

## 2. 工作区编译与环境加载

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot substation_sim
source install/setup.bash
```

可选检查：

```bash
ros2 pkg list | grep -E "substation_sim|my_robot|gazebo_ros|cartographer_ros|slam_toolbox|nav2_map_server"
which xacro
```

## 3. 启动仿真 + SLAM

优先使用 Cartographer：

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch substation_sim slam_mapping.launch.py slam_method:=cartographer start_teleop:=false
```

如果 Cartographer 不可用，切换 slam_toolbox：

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch substation_sim slam_mapping.launch.py slam_method:=slam_toolbox start_teleop:=false
```

说明：启动日志中出现 `teleop_twist_keyboard 需要TTY` 是正常提示，需要在另一个终端手动启动遥控。

## 4. 键盘遥控跑图

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
```

建图建议：

- 线速度 0.2~0.4 m/s，角速度 0.3~0.8 rad/s
- 先绕主干道闭环，再补扫分支道路
- 转弯和设备密集区减速，避免重影与漂移

推荐跑图路线（按顺序）：

1. 起点附近先原地小角度转动 1 圈，确认 `/scan` 数据稳定，再起步。
2. 沿主干道完整绕场 1 圈形成第一条闭环，不要急转和急停。
3. 回到主干道后，依次进入每条分支道路走到尽头再原路返回。
4. 设备密集区采用“慢速 + 小幅转向”通过，保证障碍物边界清晰。
5. 最后再次沿主干道绕 1 圈做全局闭环校正，再准备保存地图。

跑图过程实时判断要点：

- RViz 中地图轮廓持续变清晰，且已建区域不会大幅抖动。
- 同一设备边界不应出现双层轮廓或明显错位。
- 主路与支路交汇处应能稳定对齐，不出现“错缝”。

出现问题时的即时修正：

- 若局部重影：降低速度，回到已建好区域重新闭环一次。
- 若整体漂移：暂停前进，缓慢转向后沿主干道再跑闭环。
- 若某片区域空白：沿该区域边缘平行往返一次补扫。

## 5. TF 树验证

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tf2_tools view_frames
```

验收目标链路：

- map -> odom
- odom -> base_link
- base_link -> laser

## 6. 地图保存

建图完成后执行：

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/substation_sim/maps/substation_global
```

输出文件：

- `src/substation_sim/maps/substation_global.yaml`
- `src/substation_sim/maps/substation_global.pgm`

## 7. 地图正确性验证

先检查文件是否存在：

```bash
rviz2
cd ~/workspace/robotsoftware/AUG/ros2_ws
ls -lh src/substation_sim/maps/substation_global.yaml src/substation_sim/maps/substation_global.pgm
```

直接查看地图图片：

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
xdg-open src/substation_sim/maps/substation_global.pgm
```

在 RViz 回放已保存地图（推荐）：

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/substation_sim/maps/substation_global.yaml
```

新终端激活 map_server：

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_util lifecycle_bringup map_server
```

RViz 中设置：

- Fixed Frame: `map`
- 添加 `Map` 显示，Topic 选择 `/map`
- 左侧边栏 `/map` 下Topic下的 `Durability Policy` 改为 `Transient Local` 

质量判据：

- 道路边缘平滑连续
- 设备障碍物边界清晰
- 无明显重影或整体漂移

如果想保留灰色未知（Unknown）区域：

- 修改.pgm文件对应的.yaml文件中的 `free_thresh` 为0.15

## 8. 常见报错与处理

`package 'substation_sim' not found`：

```bash
cd ~/workspace/robotsoftware/AUG/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select substation_sim my_robot
source install/setup.bash
```

`file not found: xacro`：

```bash
sudo apt install -y ros-humble-xacro
source /opt/ros/humble/setup.bash
```

`package 'gazebo_ros' not found`：

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

`package 'cartographer_ros' not found`：

```bash
sudo apt install -y ros-humble-cartographer-ros
```

`Package 'nav2_map_server' not found`：

```bash
sudo apt install -y ros-humble-nav2-map-server
```
