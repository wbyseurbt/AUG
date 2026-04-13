#!/usr/bin/env python3
import csv
import math
import re
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def build_route_from_csv(
    csv_file: str,
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
) -> List[Tuple[str, float, float]]:
    route: List[Tuple[str, float, float]] = [
        ('START', start_x, start_y),
        ('ROAD_CRUISE_1', -30.0, -15.0),
    ]
    transformers = {}
    ins_rows = {}

    if csv_file:
        with open(csv_file, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = (row.get('name') or '').strip()
                if not name:
                    continue
                # obstacles.csv has inline comments like "... //围墙", strip them.
                if '//' in name:
                    name = name.split('//', 1)[0].strip()
                lower_name = name.lower()
                if not lower_name or 'wall' in lower_name:
                    continue

                try:
                    x = float((row.get('x') or '').strip())
                    y = float((row.get('y') or '').strip())
                except (TypeError, ValueError):
                    continue

                tf_match = re.fullmatch(r'tf_(\d+)', lower_name)
                if tf_match:
                    transformers[int(tf_match.group(1))] = (name, x, y)
                    continue

                row_match = re.fullmatch(r'ins_row_(\d+)_post_(\d+)', lower_name)
                if row_match:
                    row_idx = int(row_match.group(1))
                    post_idx = int(row_match.group(2))
                    ins_rows.setdefault(row_idx, []).append((post_idx, name, x, y))

    # 先去四个变压器
    for tf_idx in (1, 2, 3, 4):
        if tf_idx in transformers:
            route.append(transformers[tf_idx])

    # 巡检完变压器先回主路
    route.append(('ROAD_RETURN', 8.0, 0.0))

    # 再巡检六组绝缘子排，每组选中间桩位作为代表点（优先 post_3）
    for row_idx in range(1, 7):
        posts = ins_rows.get(row_idx, [])
        if not posts:
            continue
        posts.sort(key=lambda item: item[0])
        target = None
        for post_idx, name, x, y in posts:
            if post_idx == 3:
                target = (name, x, y)
                break
        if target is None:
            _, name, x, y = posts[len(posts) // 2]
            target = (name, x, y)
        route.append(target)

    route.append(('END', end_x, end_y))
    return route


class AutoGoalSender(Node):
    def __init__(self):
        super().__init__('auto_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_index = 0
        self._goals: List[Tuple[str, float, float]] = []

        self.declare_parameter('start_delay_sec', 8.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('facility_csv', '')
        self.declare_parameter('route_start_x', -35.0)
        self.declare_parameter('route_start_y', -35.0)
        self.declare_parameter('route_end_x', 45.0)
        self.declare_parameter('route_end_y', 40.0)
        self.declare_parameter('goal_dwell_sec', 2.0)

        self._frame_id = self.get_parameter('frame_id').value
        facility_csv = str(self.get_parameter('facility_csv').value)
        route_start_x = float(self.get_parameter('route_start_x').value)
        route_start_y = float(self.get_parameter('route_start_y').value)
        route_end_x = float(self.get_parameter('route_end_x').value)
        route_end_y = float(self.get_parameter('route_end_y').value)

        try:
            self._goals = build_route_from_csv(
                csv_file=facility_csv,
                start_x=route_start_x,
                start_y=route_start_y,
                end_x=route_end_x,
                end_y=route_end_y,
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load facility_csv={facility_csv}: {exc}')
            self._goals = [('START', route_start_x, route_start_y), ('END', route_end_x, route_end_y)]

        delay = float(self.get_parameter('start_delay_sec').value)
        self._goal_dwell_sec = float(self.get_parameter('goal_dwell_sec').value)
        self.get_logger().info(f'Auto navigation starts in {delay:.1f}s, total goals: {len(self._goals)}')
        self.create_timer(delay, self._on_start_timer)
        self._started = False
        self._goal_retry_timer = None
        self._goal_advance_timer = None
        self._goal_token_counter = 0
        self._current_goal_token = None

    def _on_start_timer(self):
        if self._started:
            return
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose action server is unavailable, retrying...')
            return
        self._started = True
        self.get_logger().info('navigate_to_pose action server is ready.')
        self._send_current_goal()

    def _send_current_goal(self):
        if self._goal_index >= len(self._goals):
            self.get_logger().info('All navigation goals have been completed.')
            return

        goal_name, goal_x, goal_y = self._goals[self._goal_index]
        if self._goal_index + 1 < len(self._goals):
            _, next_x, next_y = self._goals[self._goal_index + 1]
            goal_yaw = math.atan2(next_y - goal_y, next_x - goal_x)
        else:
            goal_yaw = 0.0
        sin_yaw, cos_yaw = yaw_to_quaternion(goal_yaw)

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = sin_yaw
        pose.pose.orientation.w = cos_yaw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'Sending goal #{self._goal_index + 1}: '
            f'{goal_name}, x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_yaw:.2f}'
        )
        self._goal_token_counter += 1
        token = self._goal_token_counter
        self._current_goal_token = token
        send_future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(lambda fut, t=token: self._on_goal_response(fut, t))

    def _on_goal_response(self, future, token: int):
        if token != self._current_goal_token:
            return
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal was rejected by Nav2, retrying in 3s.')
                if self._goal_retry_timer is None:
                    self._goal_retry_timer = self.create_timer(3.0, self._retry_current_goal_once)
                return
            self.get_logger().info('Goal accepted, waiting for result...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda fut, t=token: self._on_goal_done(fut, t))
        except Exception as exc:
            self.get_logger().error(f'Goal response callback exception: {exc}')
            if self._goal_retry_timer is None:
                self._goal_retry_timer = self.create_timer(3.0, self._retry_current_goal_once)

    def _retry_current_goal_once(self):
        if self._goal_retry_timer is not None:
            self._goal_retry_timer.cancel()
            self._goal_retry_timer = None
        self._send_current_goal()

    def _on_goal_done(self, future, token: int):
        if token != self._current_goal_token:
            return
        try:
            wrapped_result = future.result()
            status = getattr(wrapped_result, 'status', None)
            nav_result = getattr(wrapped_result, 'result', None)
            error_code = getattr(nav_result, 'error_code', None)
            if error_code is not None:
                self.get_logger().info(
                    f'Goal #{self._goal_index + 1} done, status={status}, nav2_error_code={error_code}'
                )
            else:
                self.get_logger().info(
                    f'Goal #{self._goal_index + 1} done, status={status}'
                )
            self.get_logger().info(
                f'Dwell {self._goal_dwell_sec:.1f}s at current goal, then continue.'
            )
            if self._goal_advance_timer is None:
                self._goal_advance_timer = self.create_timer(
                    self._goal_dwell_sec, self._advance_to_next_goal_once
                )
        except Exception as exc:
            self.get_logger().error(f'Goal done callback exception: {exc}')
            if self._goal_retry_timer is None:
                self._goal_retry_timer = self.create_timer(3.0, self._retry_current_goal_once)

    def _advance_to_next_goal_once(self):
        if self._goal_advance_timer is not None:
            self._goal_advance_timer.cancel()
            self._goal_advance_timer = None
        self._goal_index += 1
        self._send_current_goal()

    def _on_feedback(self, _feedback_msg):
        return


def main():
    rclpy.init()
    node = AutoGoalSender()
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
