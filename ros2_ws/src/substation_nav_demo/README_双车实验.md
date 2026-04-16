# substation\_nav\_demo 双车实验说明

本说明用于当前项目的双车协同实验（固定两辆车：`ugv1`、`ugv2`）。

能力覆盖：

- 双车同时 Spawn（Gazebo）
- 话题与命名空间隔离（如 `/ugv1/cmd_vel`、`/ugv2/cmd_vel`）
- 双车自动巡航（航点分配）
- 到点停留、终端打印检测信息、保存相机截图

***

## 1. 目录与入口

- 工作区：`/home/robot/AUG/ros2_ws`
- 主启动文件：`substation_nav_demo/launch/multi_ugv_patrol.launch.py`
- 双车航点文件：`substation_nav_demo/config/multi_ugv_waypoints.yaml`
- 任务控制脚本：`substation_nav_demo/scripts/multi_ugv_patrol_controller.py`
- 多车 RViz 配置：`substation_nav_demo/config/multi_nav_demo.rviz`
- 截图目录（默认）：`/home/robot/AUG/ros2_ws/screenshots`

***

## 2. 第一次或改代码后编译

```bash
cd AUG/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to substation_nav_demo
source install/setup.bash
```

***

## 3. 自动巡航模式（双车）

### 终端1：启动双车巡航

```bash
cd AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch substation_nav_demo multi_ugv_patrol.launch.py gui:=true
```

说明：

- 启动后会生成 `ugv1` 和 `ugv2`
- 启动后会自动拉起双车 Nav2（全图地图导航）
- RViz 会显示 `/ugv1/plan`、`/ugv1/local_plan`、`/ugv2/plan`、`/ugv2/local_plan` 轨迹
- 终端会打印类似 `正在检测设备 XXX...`
- 到点会在 `screenshots` 目录保存图片（PPM 格式）

***

## 3.1 手动控制模式（双车调试，含地图加载）

按下面步骤执行，可进入“加载地图 + 双车手动驾驶”模式。

### 第1步：终端1启动双车并加载地图

```bash
cd AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch substation_nav_demo multi_ugv_patrol.launch.py \
  gui:=true \
  map:=/home/robot/AUG/ros2_ws/src/substation_sim/maps/substation_edited.yaml
```

### 第2步：关闭自动巡航控制器（避免抢速度指令）

新开终端2执行：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/AUG/ros2_ws/install/setup.bash
pkill -f multi_ugv_patrol_controller.py
```

### 第3步：终端3手动控制 ugv1

```bash
source /opt/ros/humble/setup.bash
source /home/robot/AUG/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv1/cmd_vel
```

### 第4步：终端4手动控制 ugv2

```bash
source /opt/ros/humble/setup.bash
source /home/robot/AUG/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv2/cmd_vel
```

说明：

- 双车手动模式不经过 `cmd_vel_repeater`，键盘命令直接发到各自 `/ugvX/cmd_vel`。
- 只想手动接管一台车时，只开对应一个 teleop 终端即可。
- 恢复自动巡航：重启 launch，或手动重新启动 `/multi_ugv_patrol_controller`。

***

## 4. 业务逻辑闭环（自主巡航）

- `ugv1`：从单车起点 `(-35, -35)` 出发，按固定顺序巡检变电站设备点列，最后返回起点。
- `ugv2`：从单车终点 `(45, 40)` 出发，按固定顺序巡检电线杆点列，最后返回起点。
- 到达每个设备点后默认停留 `4.0s`（可通过 `dwell_sec` 调整到 3\~5 秒）。
- 停留期间终端打印 `正在检测设备 X...`。
- 每个设备点停留时自动保存一张车载摄像头截图到本地目录。

可选参数示例：

```bash
ros2 launch substation_nav_demo multi_ugv_patrol.launch.py \
  gui:=true dwell_sec:=4.0 capture_dir:=/home/robot/AUG/ros2_ws/screenshots
```

***

## 5. 结果验收命令

### 5.1 验证只存在双车话题

```bash
source /opt/ros/humble/setup.bash
source /home/robot/AUG/ros2_ws/install/setup.bash
ros2 topic list | grep -E '^/ugv(1|2)/(cmd_vel|odom|scan|camera/image_raw)$|^/ugv3/'
```

预期：

- 有 `/ugv1/...` 和 `/ugv2/...`
- 不应有 `/ugv3/...`

### 5.2 查看截图是否生成

```bash
ls -lh /home/robot/AUG/ros2_ws/screenshots
```

***

## 6. 常见问题

### 6.1 `Read-only file system`（日志/配置写入失败）

确保每个终端都设置了：

```bash
export ROS_LOG_DIR=/tmp/ros_log && mkdir -p /tmp/ros_log
export GAZEBO_HOME=/tmp/gazebo && mkdir -p /tmp/gazebo
export GAZEBO_LOG_PATH=/tmp/gazebo_log && mkdir -p /tmp/gazebo_log
```

### 6.2 Gazebo 端口冲突（Address already in use）

关闭残留 Gazebo 进程后重启：

```bash
pkill -f gzserver
pkill -f gzclient
```

### 6.3 到点不截图

- 检查是否有相机话题：`/ugv1/camera/image_raw`、`/ugv2/camera/image_raw`
- 确认 `capture_dir` 目录可写
- 终端若提示编码不支持，可改相机输出到 `rgb8` 或 `bgr8`
