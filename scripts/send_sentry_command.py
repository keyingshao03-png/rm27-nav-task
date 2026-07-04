#!/usr/bin/env python3
"""Publish one simulated referee command to /sentry_command."""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "start"
    rclpy.init()
    node = Node("demo_sentry_command_sender")
    pub = node.create_publisher(String, "/sentry_command", 10)

    msg = String()
    msg.data = command
    deadline = time.time() + 1.5
    while time.time() < deadline:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(f"Published /sentry_command: {command}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
