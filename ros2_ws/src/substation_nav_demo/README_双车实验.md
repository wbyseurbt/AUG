# substation_nav_demo 双车实验说明

本说明用于当前项目的双车协同实验（固定两辆车：`ugv1`、`ugv2`）。

能力覆盖：

- 双车同时 Spawn（Gazebo）
- 话题与命名空间隔离（如 `/ugv1/cmd_vel`、`/ugv2/cmd_vel`）
- 双车自动巡航（航点分配）
- 到点停留、终端打印检测信息、保存相机截图
- 键盘分别控制两辆车

---

## 1. 目录与入口

- 工作区：`/home/zzz/AUG/ros2_ws`
- 主启动文件：`substation_nav_demo/launch/multi_ugv_patrol.launch.py`
- 双车航点文件：`substation_nav_demo/config/multi_ugv_waypoints.yaml`
- 任务控制脚本：`substation_nav_demo/scripts/multi_ugv_patrol_controller.py`
- 截图目录（默认）：`/home/zzz/AUG/ros2_ws/screenshots`

---

## 2. 第一次或改代码后编译

```bash
cd /home/zzz/AUG/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select substation_nav_demo
source install/setup.bash
```

---

## 3. 自动巡航模式（双车）

### 终端1：启动双车巡航

```bash
cd /home/zzz/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_LOG_DIR=/tmp/ros_log && mkdir -p /tmp/ros_log
export GAZEBO_HOME=/tmp/gazebo && mkdir -p /tmp/gazebo
export GAZEBO_LOG_PATH=/tmp/gazebo_log && mkdir -p /tmp/gazebo_log

ros2 launch substation_nav_demo multi_ugv_patrol.launch.py gui:=true
```

说明：

- 启动后会生成 `ugv1` 和 `ugv2`
- 终端会打印类似 `正在检测设备 XXX...`
- 到点会在 `screenshots` 目录保存图片（PPM 格式）

---

## 4. 键盘分别控制两辆车

注意：`multi_ugv_patrol.launch.py` 会自动发布速度命令。  
如果你想“纯键盘控制”，请先停止自动巡航启动（终端1 `Ctrl+C`），再按下面方式启动仅双车场景。

### 4.1 终端1：仅启动双车场景（不启任务控制）

```bash
cd /home/zzz/AUG/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_LOG_DIR=/tmp/ros_log && mkdir -p /tmp/ros_log
export GAZEBO_HOME=/tmp/gazebo && mkdir -p /tmp/gazebo
export GAZEBO_LOG_PATH=/tmp/gazebo_log && mkdir -p /tmp/gazebo_log

ros2 launch my_robot multi_spawn.launch.py
```

### 4.2 终端2：控制 ugv1

```bash
source /opt/ros/humble/setup.bash
source /home/zzz/AUG/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv1/cmd_vel
```

### 4.3 终端3：控制 ugv2

```bash
source /opt/ros/humble/setup.bash
source /home/zzz/AUG/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv2/cmd_vel
```

说明：

- 两个 teleop 终端按键完全一样
- 哪个终端有焦点，就控制哪辆车

---

## 5. 常用键位（teleop_twist_keyboard）

- `i` 前进
- `,` 后退
- `j` 原地左转
- `l` 原地右转
- `u` 前进左转
- `o` 前进右转
- `m` 后退左转
- `.` 后退右转
- `k` 停车
- `q/z` 同时增/减线速度和角速度
- `w/x` 增/减线速度
- `e/c` 增/减角速度

---

## 6. 结果验收命令

### 6.1 验证只存在双车话题

```bash
source /opt/ros/humble/setup.bash
source /home/zzz/AUG/ros2_ws/install/setup.bash
ros2 topic list | grep -E '^/ugv(1|2)/(cmd_vel|odom|scan|camera/image_raw)$|^/ugv3/'
```

预期：

- 有 `/ugv1/...` 和 `/ugv2/...`
- 不应有 `/ugv3/...`

### 6.2 查看截图是否生成

```bash
ls -lh /home/zzz/AUG/ros2_ws/screenshots
```

---

## 7. 常见问题

### 7.1 `Read-only file system`（日志/配置写入失败）

确保每个终端都设置了：

```bash
export ROS_LOG_DIR=/tmp/ros_log && mkdir -p /tmp/ros_log
export GAZEBO_HOME=/tmp/gazebo && mkdir -p /tmp/gazebo
export GAZEBO_LOG_PATH=/tmp/gazebo_log && mkdir -p /tmp/gazebo_log
```

### 7.2 Gazebo 端口冲突（Address already in use）

关闭残留 Gazebo 进程后重启：

```bash
pkill -f gzserver
pkill -f gzclient
```

### 7.3 键盘按了车不动

- 确认焦点在对应 teleop 终端
- 检查 remap 是否正确（`/ugv1/cmd_vel` 或 `/ugv2/cmd_vel`）
- 不要和自动巡航同时运行（会抢 `cmd_vel`）

