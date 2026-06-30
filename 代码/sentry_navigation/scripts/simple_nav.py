#!/usr/bin/env python3
"""
简易导航器 v2 — 基于 AMCL 定位 + 路径跟踪
【降级方案】当 Nav2 不稳定时使用。优先使用 Nav2 完整栈。

改进点：
1. 使用 /amcl_pose 获取真实的 map 坐标系位姿（而非 odom 漂移）
2. 支持通过 /goal_point 发布目标点
3. 前方障碍检测 + 绕行 / 等待
4. 到达容忍度可配置
"""
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped


class SimpleNav(Node):
    def __init__(self):
        super().__init__('simple_nav')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PointStamped, '/goal_point', self.goal_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # 优先使用 AMCL 位姿，降级使用 odom
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.rx = self.ry = self.ryaw = 0.0
        self.gx = self.gy = None
        self.has_amcl = False  # 是否接收到 AMCL 数据

        # 激光数据
        self.ranges = []
        self.front_dist = 99.0
        self.left_dist = 99.0
        self.right_dist = 99.0

        # 控制参数
        self.obstacle_threshold = 0.5   # 障碍距离阈值 (m)
        self.goal_tolerance = 0.2        # 到达容忍度 (m)
        self.max_linear = 0.3            # 最大线速度
        self.max_angular = 0.8           # 最大角速度
        self.stuck_time = 0.0            # 卡住计时
        self.last_pos = (0.0, 0.0)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('SimpleNav v2 ready (AMCL + obstacle avoidance)')

    def amcl_cb(self, msg):
        self.has_amcl = True
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.ryaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def odom_cb(self, msg):
        if not self.has_amcl:
            self.rx = msg.pose.pose.position.x
            self.ry = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.ryaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def scan_cb(self, msg):
        self.ranges = msg.ranges
        n = len(msg.ranges)
        if n > 100:
            # 前方 ±15° ≈ 前方 30° 扇形
            front_n = int(n * 15 / 360)
            self.front_dist = min(
                min(msg.ranges[0:front_n] if msg.ranges[0:front_n] else [99.0]),
                min(msg.ranges[-front_n:] if msg.ranges[-front_n:] else [99.0])
            )
            # 左方 60°~120°
            self.left_dist = min(msg.ranges[n//6: n//3]) if n > 200 else 99.0
            # 右方 240°~300°
            self.right_dist = min(msg.ranges[2*n//3: 5*n//6]) if n > 200 else 99.0

    def goal_cb(self, msg):
        self.gx = msg.point.x
        self.gy = msg.point.y
        self.stuck_time = 0.0
        self.last_pos = (self.rx, self.ry)
        self.get_logger().info(f'New goal: ({self.gx:.2f}, {self.gy:.2f})')

    def control_loop(self):
        if self.gx is None:
            self.cmd_pub.publish(Twist())  # 停止
            return

        dx = self.gx - self.rx
        dy = self.gy - self.ry
        dist_to_goal = math.hypot(dx, dy)

        # 到达目标
        if dist_to_goal < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'Goal reached! dist={dist_to_goal:.3f}m')
            self.gx = None
            return

        # 卡住检测: 5 秒内移动不到 0.1m
        moved = math.hypot(self.rx - self.last_pos[0], self.ry - self.last_pos[1])
        if moved < 0.02:
            self.stuck_time += 0.1
        else:
            self.stuck_time = 0.0
            self.last_pos = (self.rx, self.ry)

        # 目标角度
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - self.ryaw),
                             math.cos(target_yaw - self.ryaw))

        twist = Twist()

        # ---- 避障逻辑 ----
        if self.front_dist < self.obstacle_threshold:
            # 前方有障碍：先后退再旋转
            if self.stuck_time > 2.0:
                self.get_logger().warn(f'Blocked! front={self.front_dist:.2f}m, backing up')
                twist.linear.x = -0.2   # 后退
                twist.angular.z = 0.5   # 边退边转
            elif self.left_dist > self.right_dist:
                twist.angular.z = self.max_angular * 1.5  # 更快左转
                twist.linear.x = 0.0
            else:
                twist.angular.z = -self.max_angular * 1.5 # 更快右转
                twist.linear.x = 0.0

        elif self.stuck_time > 5.0:
            # 长时间卡住：后退 + 旋转
            self.get_logger().warn(f'Stuck for {self.stuck_time:.1f}s, recovery!')
            twist.linear.x = -0.15  # 后退
            twist.angular.z = 0.5   # 同时旋转
            self.stuck_time = 0.0

        elif abs(yaw_err) > 0.3:
            # 朝向偏差大：原地旋转
            twist.angular.z = self.max_angular if yaw_err > 0 else -self.max_angular
            twist.linear.x = 0.0

        else:
            # 正常前进：速度与距离成正比（防止超调）
            speed = min(self.max_linear, dist_to_goal * 0.5)
            twist.linear.x = max(0.05, speed)  # 不低于 0.05 m/s
            twist.angular.z = yaw_err * 0.5

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    rclpy.spin(SimpleNav())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
