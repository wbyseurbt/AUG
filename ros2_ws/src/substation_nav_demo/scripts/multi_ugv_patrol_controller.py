#!/usr/bin/env python3
import math
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image


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
    waiting_for_dwell: bool = False
    dwell_end_sec: float = 0.0
    in_flight: bool = False
    last_goal_token: int = 0
    retry_after_sec: float = 0.0
    finished: bool = False
    snapshot_taken_at_index: int = -1


class MultiUgvPatrolController(Node):
    def __init__(self):
        super().__init__('multi_ugv_patrol_controller')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('dwell_sec', 4.0)
        self.declare_parameter('capture_dir', '/tmp/ugv_screenshots')
        self.declare_parameter('start_delay_sec', 12.0)
        self.declare_parameter('goal_retry_sec', 2.0)
        self.declare_parameter('skip_start_waypoint', True)

        self.robot_names = ['ugv1', 'ugv2']
        self.frame_id = str(self.get_parameter('frame_id').value)
        requested_dwell_sec = float(self.get_parameter('dwell_sec').value)
        # Business requirement: per-point inspection dwell time is 3-5 seconds.
        self.dwell_sec = min(5.0, max(3.0, requested_dwell_sec))
        self.goal_retry_sec = float(self.get_parameter('goal_retry_sec').value)
        start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.start_time_sec = self.get_clock().now().nanoseconds / 1e9 + start_delay_sec

        capture_dir = str(self.get_parameter('capture_dir').value)
        self.capture_dir = Path(os.path.expanduser(capture_dir))
        self.capture_dir.mkdir(parents=True, exist_ok=True)

        self.waypoints = self._load_waypoints(str(self.get_parameter('waypoints_file').value))
        self.states: Dict[str, RobotState] = {name: RobotState() for name in self.robot_names}
        skip_start_waypoint = bool(self.get_parameter('skip_start_waypoint').value)
        if skip_start_waypoint:
            for robot in self.robot_names:
                route = self.waypoints.get(robot, [])
                if len(route) >= 2 and route[0].name.upper().endswith('START'):
                    self.states[robot].index = 1
                    self.get_logger().info(f'{robot}: 跳过起点航点，直接前往第一个巡检点')
        self.action_clients = {
            name: ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')
            for name in self.robot_names
        }
        self.last_images: Dict[str, Optional[Image]] = {name: None for name in self.robot_names}

        for name in self.robot_names:
            self.create_subscription(Image, f'/{name}/camera/image_raw', self._image_callback_builder(name), 10)

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f'Multi-UGV patrol started, robots={self.robot_names}, frame={self.frame_id}, '
            f'dwell={self.dwell_sec:.1f}s, capture_dir={self.capture_dir}, start_delay={start_delay_sec:.1f}s'
        )
        if abs(self.dwell_sec - requested_dwell_sec) > 1e-6:
            self.get_logger().warn(
                f'dwell_sec={requested_dwell_sec:.2f} 超出业务范围，已限制到 {self.dwell_sec:.1f}s'
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

    def _build_goal(self, robot: str) -> NavigateToPose.Goal:
        state = self.states[robot]
        points = self.waypoints[robot]
        target = points[state.index]
        if state.index + 1 < len(points):
            next_point = points[state.index + 1]
            yaw = math.atan2(next_point.y - target.y, next_point.x - target.x)
        else:
            yaw = 0.0

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target.x
        pose.pose.position.y = target.y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _send_goal(self, robot: str, now_sec: float):
        state = self.states[robot]
        points = self.waypoints.get(robot, [])
        if state.finished or state.in_flight or state.waiting_for_dwell:
            return
        if not points:
            state.finished = True
            self.get_logger().warn(f'{robot}: 航点为空，任务结束')
            return
        if state.index >= len(points):
            state.finished = True
            self.get_logger().info(f'{robot}: 全部航点完成')
            return
        if now_sec < state.retry_after_sec:
            return

        client = self.action_clients[robot]
        if not client.wait_for_server(timeout_sec=0.05):
            return

        target = points[state.index]
        goal = self._build_goal(robot)
        state.in_flight = True
        state.last_goal_token += 1
        token = state.last_goal_token
        self.get_logger().info(
            f'{robot}: 发送航点 {state.index + 1}/{len(points)} {target.name} '
            f'({target.x:.2f}, {target.y:.2f})'
        )
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda fut, r=robot, t=token: self._on_goal_response(r, t, fut))

    def _on_goal_response(self, robot: str, token: int, future):
        state = self.states[robot]
        if token != state.last_goal_token:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                state.in_flight = False
                state.retry_after_sec = self.get_clock().now().nanoseconds / 1e9 + self.goal_retry_sec
                self.get_logger().warn(f'{robot}: Nav2 拒绝航点，稍后重试')
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda fut, r=robot, t=token: self._on_goal_done(r, t, fut))
        except Exception as exc:
            state.in_flight = False
            state.retry_after_sec = self.get_clock().now().nanoseconds / 1e9 + self.goal_retry_sec
            self.get_logger().warn(f'{robot}: 航点发送异常 {exc}，稍后重试')

    def _on_goal_done(self, robot: str, token: int, future):
        state = self.states[robot]
        if token != state.last_goal_token:
            return

        state.in_flight = False
        now_sec = self.get_clock().now().nanoseconds / 1e9
        try:
            wrapped_result = future.result()
            status = getattr(wrapped_result, 'status', None)
        except Exception as exc:
            state.retry_after_sec = now_sec + self.goal_retry_sec
            self.get_logger().warn(f'{robot}: 航点结果异常 {exc}，稍后重试')
            return

        # GoalStatus.STATUS_SUCCEEDED = 4
        if status != 4:
            state.retry_after_sec = now_sec + self.goal_retry_sec
            self.get_logger().warn(f'{robot}: 航点失败 status={status}，稍后重试')
            return

        points = self.waypoints[robot]
        target = points[state.index]
        self.get_logger().info(f'{robot}: 正在检测设备 {target.device}...')
        if state.snapshot_taken_at_index != state.index:
            self._save_snapshot(robot, target)
            state.snapshot_taken_at_index = state.index
        state.waiting_for_dwell = True
        state.dwell_end_sec = now_sec + self.dwell_sec

    def _control_loop(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self.start_time_sec:
            return

        for robot in self.robot_names:
            state = self.states[robot]
            if state.finished:
                continue

            if state.waiting_for_dwell:
                if now_sec >= state.dwell_end_sec:
                    state.waiting_for_dwell = False
                    state.index += 1
                    if state.index >= len(self.waypoints.get(robot, [])):
                        state.finished = True
                        self.get_logger().info(f'{robot}: 全部航点完成')
                continue

            self._send_goal(robot, now_sec)


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
