#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfRepublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_republisher')
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.get_logger().info('Odom TF republisher started, odom -> base frame bridge enabled')

    def _on_odom(self, msg: Odometry):
        # Bridge odom topic pose to TF tree so Nav2 can always resolve odom/base frames.
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfRepublisher()
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
