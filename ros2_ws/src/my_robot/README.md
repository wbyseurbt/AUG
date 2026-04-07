# my_robot 一步一步启动说明

当前只使用一个工作区：`/home/robot/ros2_ws`。  
本说明适用于以下两个包：

- 环境包：`substation_sim`
- 小车包：`my_robot`

统一启动入口：`my_robot/launch/substation_combo.launch.py`

## 1. 第一次使用先编译

```bash
cd /home/robot/ros2_ws
colcon build --packages-select substation_sim my_robot
```

## 2. 每个新终端先执行环境加载

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/ros2_ws/install/setup.bash
```

如果你的 `~/.ros` 无写权限，再执行：

```bash
export ROS_LOG_DIR=/tmp/ros_log
mkdir -p /tmp/ros_log
```

## 3. 推荐启动方式（三终端）

### 第1步（终端1）：启动环境并生成小车

```bash
ros2 launch my_robot substation_combo.launch.py start_teleop:=false
```

### 第2步（终端2）：键盘控制小车

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
```

键位：

- `i` 前进
- `,` 后退
- `j` 左转
- `l` 右转
- `u/o` 前进同时转向
- `m/.` 后退同时转向
- `k` 停止

### 第3步（终端3）：打开 RViz 看雷达和相机

```bash
ros2 run rviz2 rviz2 -d /home/robot/ros2_ws/src/my_robot/rviz/diffbot.rviz --ros-args -p use_sim_time:=true
```

## 4. 快速自检

查看话题是否正常：

```bash
ros2 topic list | grep -E 'scan|camera|image|odom|cmd_vel'
```

查看关键数据：

```bash
ros2 topic echo /scan
ros2 topic echo /camera/image_raw
ros2 topic echo /odom
```

## 5. 常用启动参数

```bash
ros2 launch my_robot substation_combo.launch.py gui:=false
ros2 launch my_robot substation_combo.launch.py spawn_x:=1.0 spawn_y:=2.0 spawn_z:=0.1
ros2 launch my_robot substation_combo.launch.py entity_name:=diffbot
```

## 6. 常见问题

### 6.1 键盘按了不动

- 确认输入焦点在终端2
- 终端1使用 `start_teleop:=false`，避免重复 teleop
- 执行 `ros2 topic echo /cmd_vel_teleop` 确认有按键指令

### 6.2 RViz 显示异常或时间不同步

- 启动 RViz 时使用 `use_sim_time:=true`
- 不要重复启动多个 Gazebo 或 `robot_state_publisher`

### 6.3 编译报旧路径缓存错误

```bash
rm -rf /home/robot/ros2_ws/build /home/robot/ros2_ws/install /home/robot/ros2_ws/log
cd /home/robot/ros2_ws
colcon build
```
