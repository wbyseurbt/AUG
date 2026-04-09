#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="in_topic", default="cmd_vel_teleop")
    parser.add_argument("--out", dest="out_topic", default="cmd_vel")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--timeout", type=float, default=0.2)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = rclpy.create_node("cmd_vel_repeater")

    last_msg = Twist()
    last_time = 0.0
    has_input = False
    is_active = False

    def cb(msg: Twist):
        nonlocal last_msg, last_time, has_input
        last_msg = msg
        last_time = time.monotonic()
        has_input = True

    sub_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )
    pub_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )
    pub = node.create_publisher(Twist, args.out_topic, pub_qos)
    node.create_subscription(Twist, args.in_topic, cb, sub_qos)

    period = 1.0 / float(args.rate)
    stop = Twist()

    try:
        while rclpy.ok():
            nonlocal_active = is_active
            rclpy.spin_once(node, timeout_sec=0.0)
            if has_input and (time.monotonic() - last_time <= float(args.timeout)):
                pub.publish(last_msg)
                nonlocal_active = True
            elif nonlocal_active:
                pub.publish(stop)
                nonlocal_active = False
            is_active = nonlocal_active
            time.sleep(period)
    finally:
        pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
