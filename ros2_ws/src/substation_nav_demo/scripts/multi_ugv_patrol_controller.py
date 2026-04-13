#!/usr/bin/env python3
import math
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _to_rgb_bytes(image: Image) -> Optional[bytes]:
    data = bytes(image.data)
    if image.encoding == 'rgb8':
        return data

    if image.encoding == 'bgr8':
        out = bytearray(len(data))
        for i in range(0, len(data), 3):
            out[i] = data[i + 2]
            out[i + 1] = data[i + 1]
            out[i + 2] = data[i]
        return bytes(out)

    return None


@dataclass
class PatrolPoint:
    name: str
    x: float
    y: float
    device: str


@dataclass
class RobotState:
    index: int = 0
    dwell_end_sec: float = 0.0
    waiting_for_dwell: bool = False
    finished: bool = False
    snapshot_taken_at_index: int = -1


class MultiUgvPatrolController(Node):
    def __init__(self):
        super().__init__('multi_ugv_patrol_controller')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('dwell_sec', 4.0)
        self.declare_parameter('capture_dir', '/tmp/ugv_screenshots')
        self.declare_parameter('goal_tolerance', 0.7)
        self.declare_parameter('yaw_tolerance', 0.35)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 0.9)
        self.declare_parameter('k_linear', 0.45)
        self.declare_parameter('k_angular', 1.2)

        self.robot_names = ['ugv1', 'ugv2']
        self.dwell_sec = float(self.get_parameter('dwell_sec').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.k_angular = float(self.get_parameter('k_angular').value)

        capture_dir = str(self.get_parameter('capture_dir').value)
        self.capture_dir = Path(os.path.expanduser(capture_dir))
        self.capture_dir.mkdir(parents=True, exist_ok=True)

        self.waypoints = self._load_waypoints(str(self.get_parameter('waypoints_file').value))
        self.states: Dict[str, RobotState] = {name: RobotState() for name in self.robot_names}

        self.model_indices: Dict[str, int] = {}
        self.model_poses: Dict[str, tuple] = {}
        self.last_images: Dict[str, Optional[Image]] = {name: None for name in self.robot_names}

        self.cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in self.robot_names
        }

        self.create_subscription(ModelStates, '/gazebo/model_states', self._on_model_states, 10)
        for name in self.robot_names:
            self.create_subscription(Image, f'/{name}/camera/image_raw', self._image_callback_builder(name), 10)

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f'Multi-UGV patrol started, robots={self.robot_names}, dwell={self.dwell_sec:.1f}s, '
            f'capture_dir={self.capture_dir}'
        )

    def _load_waypoints(self, file_path: str) -> Dict[str, List[PatrolPoint]]:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = yaml.safe_load(f) or {}

        routes = content.get('robots', {})
        loaded: Dict[str, List[PatrolPoint]] = {}
        for robot in self.robot_names:
            points = []
            for p in routes.get(robot, []):
                points.append(
                    PatrolPoint(
                        name=str(p.get('name', 'UNKNOWN')),
                        x=float(p.get('x', 0.0)),
                        y=float(p.get('y', 0.0)),
                        device=str(p.get('device', p.get('name', 'UNKNOWN'))),
                    )
                )
            loaded[robot] = points
            self.get_logger().info(f'{robot} loaded waypoints: {len(points)}')
        return loaded

    def _image_callback_builder(self, robot: str):
        def _cb(msg: Image):
            self.last_images[robot] = msg
        return _cb

    def _on_model_states(self, msg: ModelStates):
        for i, model_name in enumerate(msg.name):
            if model_name in self.states:
                self.model_indices[model_name] = i
                pose = msg.pose[i]
                yaw = _yaw_from_quaternion(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                self.model_poses[model_name] = (pose.position.x, pose.position.y, yaw)

    def _save_snapshot(self, robot: str, point: PatrolPoint):
        image = self.last_images.get(robot)
        if image is None:
            self.get_logger().warn(f'{robot}: 无可用相机帧，跳过截图')
            return

        rgb = _to_rgb_bytes(image)
        if rgb is None:
            self.get_logger().warn(f'{robot}: 不支持图像编码 {image.encoding}，跳过截图')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_device = point.device.replace(' ', '_')
        file_name = f'{robot}_{safe_device}_{timestamp}.ppm'
        file_path = self.capture_dir / file_name

        header = f'P6\n{image.width} {image.height}\n255\n'.encode('ascii')
        with open(file_path, 'wb') as f:
            f.write(header)
            f.write(rgb)
        self.get_logger().info(f'{robot}: 已保存截图 -> {file_path}')

    def _stop_robot(self, robot: str):
        self.cmd_pubs[robot].publish(Twist())

    def _control_loop(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        for robot in self.robot_names:
            state = self.states[robot]
            points = self.waypoints.get(robot, [])
            pose = self.model_poses.get(robot)

            if state.finished:
                self._stop_robot(robot)
                continue
            if not points or pose is None:
                continue

            if state.index >= len(points):
                state.finished = True
                self._stop_robot(robot)
                self.get_logger().info(f'{robot}: 全部航点完成')
                continue

            target = points[state.index]
            x, y, yaw = pose
            dx = target.x - x
            dy = target.y - y
            distance = math.hypot(dx, dy)
            heading = math.atan2(dy, dx)
            yaw_error = _normalize_angle(heading - yaw)

            if distance < self.goal_tolerance and abs(yaw_error) < self.yaw_tolerance:
                self._stop_robot(robot)
                if not state.waiting_for_dwell:
                    state.waiting_for_dwell = True
                    state.dwell_end_sec = now_sec + self.dwell_sec
                    self.get_logger().info(f'{robot}: 正在检测设备 {target.device}...')
                    if state.snapshot_taken_at_index != state.index:
                        self._save_snapshot(robot, target)
                        state.snapshot_taken_at_index = state.index
                elif now_sec >= state.dwell_end_sec:
                    state.waiting_for_dwell = False
                    state.index += 1
                continue

            state.waiting_for_dwell = False
            cmd = Twist()
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, self.k_angular * yaw_error),
            )
            if abs(yaw_error) > 0.7:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = max(
                    0.0,
                    min(self.max_linear_speed, self.k_linear * distance),
                )
            self.cmd_pubs[robot].publish(cmd)


def main():
    rclpy.init()
    node = MultiUgvPatrolController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
