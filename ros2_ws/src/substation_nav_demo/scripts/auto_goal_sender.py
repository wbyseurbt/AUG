#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class AutoGoalSender(Node):
    def __init__(self):
        super().__init__('auto_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_index = 0
        self._goals = []

        self.declare_parameter('start_delay_sec', 8.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_1_x', 6.0)
        self.declare_parameter('goal_1_y', 0.0)
        self.declare_parameter('goal_1_yaw', 0.0)
        self.declare_parameter('goal_2_x', 0.0)
        self.declare_parameter('goal_2_y', 0.0)
        self.declare_parameter('goal_2_yaw', 3.14)
        self.declare_parameter('enable_second_goal', True)

        self._frame_id = self.get_parameter('frame_id').value
        self._goals.append((
            float(self.get_parameter('goal_1_x').value),
            float(self.get_parameter('goal_1_y').value),
            float(self.get_parameter('goal_1_yaw').value),
        ))

        if bool(self.get_parameter('enable_second_goal').value):
            self._goals.append((
                float(self.get_parameter('goal_2_x').value),
                float(self.get_parameter('goal_2_y').value),
                float(self.get_parameter('goal_2_yaw').value),
            ))

        delay = float(self.get_parameter('start_delay_sec').value)
        self.get_logger().info(f'Auto navigation starts in {delay:.1f}s, total goals: {len(self._goals)}')
        self.create_timer(delay, self._on_start_timer)
        self._started = False

    def _on_start_timer(self):
        if self._started:
            return
        self._started = True
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server is unavailable.')
            return
        self._send_current_goal()

    def _send_current_goal(self):
        if self._goal_index >= len(self._goals):
            self.get_logger().info('All navigation goals have been completed.')
            return

        goal_x, goal_y, goal_yaw = self._goals[self._goal_index]
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
            f'x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_yaw:.2f}'
        )
        send_future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2.')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_done)

    def _on_goal_done(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal #{self._goal_index + 1} done, Nav2 result code={result.error_code}')
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
