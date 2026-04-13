#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
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
    _ = csv_file
    route: List[Tuple[str, float, float]] = [
        ('START', start_x, start_y),
        ('ROAD_CRUISE_1', -30.0, -15.0),
        # 变压器是 3x3 方块，x 向外偏移约 3.2m，避免目标落在模型内部
        ('tf_4', -13.2, -10.0),
        ('tf_1', -13.2, 0.0),
        ('tf_2', -13.2, 10.0),
        ('tf_3', -13.2, 20.0),
        ('ROAD_RETURN', -13.2, 35.0),
        ('ROAD_RETURN2', 20.0, 35.0),
        # 绝缘子桩是细圆柱，x 向道路侧偏移约 1.2m，降低不可达概率
        ('ins_row_1_post_3', 23.8, 20.0),
        ('ins_row_2_post_3', 23.8, 0.0),
        ('ins_row_3_post_3', 23.8, -20.0),
        ('ins_row_6_post_3', 43.8, -20.0),
        ('ins_row_5_post_3', 43.8, 0.0),
        ('ins_row_4_post_3', 43.8, 20.0),
        ('END', end_x, end_y),
    ]
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
        self.declare_parameter('enable_stuck_recovery', True)
        self.declare_parameter('backup_speed', 0.12)
        self.declare_parameter('backup_duration_sec', 1.5)
        self.declare_parameter('retry_delay_after_backup_sec', 0.5)

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
        self._enable_stuck_recovery = bool(self.get_parameter('enable_stuck_recovery').value)
        self._backup_speed = float(self.get_parameter('backup_speed').value)
        self._backup_duration_sec = float(self.get_parameter('backup_duration_sec').value)
        self._retry_delay_after_backup_sec = float(self.get_parameter('retry_delay_after_backup_sec').value)
        self.get_logger().info(f'Auto navigation starts in {delay:.1f}s, total goals: {len(self._goals)}')
        self.create_timer(delay, self._on_start_timer)
        self.create_timer(0.1, self._backup_tick)
        self._started = False
        self._goal_retry_timer = None
        self._goal_advance_timer = None
        self._goal_token_counter = 0
        self._current_goal_token = None
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._is_backing = False
        self._backup_end_time_sec = 0.0
        self._retry_after_backup_pending = False

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
                self.get_logger().error('Goal was rejected by Nav2.')
                self._handle_goal_failure('rejected')
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
            # GoalStatus.STATUS_SUCCEEDED = 4
            if status != 4:
                self._handle_goal_failure(f'status={status}')
                return
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

    def _handle_goal_failure(self, reason: str):
        if not self._enable_stuck_recovery:
            self.get_logger().warn(f'Goal failed ({reason}), retrying in 3s.')
            if self._goal_retry_timer is None:
                self._goal_retry_timer = self.create_timer(3.0, self._retry_current_goal_once)
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._is_backing = True
        self._backup_end_time_sec = now_sec + self._backup_duration_sec
        self._retry_after_backup_pending = True
        self.get_logger().warn(
            f'Goal failed ({reason}), backing up for {self._backup_duration_sec:.1f}s at '
            f'{self._backup_speed:.2f}m/s before retry.'
        )

    def _backup_tick(self):
        if not self._is_backing:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self._backup_end_time_sec:
            cmd = Twist()
            cmd.linear.x = -abs(self._backup_speed)
            cmd.angular.z = 0.0
            self._cmd_pub.publish(cmd)
            return

        self._is_backing = False
        stop = Twist()
        self._cmd_pub.publish(stop)
        if self._retry_after_backup_pending and self._goal_retry_timer is None:
            self._retry_after_backup_pending = False
            self._goal_retry_timer = self.create_timer(
                self._retry_delay_after_backup_sec, self._retry_current_goal_once
            )

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
