#!/usr/bin/env python3
"""
odom → base_footprint TF 发布器
用于 Nav2 未启动时补全 TF 链路

注意：如果 AMCL 已在运行，它自动发布 map→odom→base_footprint，
此节点不再需要。仅在调试或降级运行时使用。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.get_logger().info('Publishing odom -> base_footprint TF')

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTfPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
