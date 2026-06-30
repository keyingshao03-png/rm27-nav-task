#!/usr/bin/env python3
"""
哨兵比赛行为决策 v2 — 完整状态机 + 自动巡逻
==============================================
状态: INIT → PATROL → CHASE → RETREAT → AVOID
输入: /odom, /scan, /sentry_command, 内部计时器
输出: /goal_point (导航目标), /sentry_state (当前状态)
"""
import rclpy
import math
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


class SentryBehavior(Node):
    def __init__(self):
        super().__init__('sentry_behavior')

        # 发布导航目标和状态
        self.goal_pub = self.create_publisher(PointStamped, '/goal_point', 10)
        self.state_pub = self.create_publisher(String, '/sentry_state', 10)

        # 订阅 (使用 AMCL 获取 map 坐标系位姿)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/sentry_command', self.cmd_cb, 10)

        # ---- 比赛参数 ----
        self.robot_hp = 100.0           # 当前血量 (100~0)
        self.remaining_time = 300.0     # 剩余时间 (秒)
        self.enemy_detected = False     # 是否发现敌方
        self.match_started = False      # 比赛是否开始

        # ---- 巡逻点 (蓝方) ----
        self.patrol_pts = [
            (-3.0, -5.0),    # 蓝方出生点
            (0.0, 0.0),      # 中央区域
            (2.0, 3.0),      # 右上方
            (-1.0, 4.0),     # 左上方
            (1.0, -1.0),     # 右下
        ]
        self.patrol_idx = 0

        # ---- 状态变量 ----
        self.state = 'INIT'
        self.rx = self.ry = 0.0
        self.front_dist = 99.0
        self.last_goal_time = 0.0
        self.state_start_time = time.time()
        self.avoid_timer = 0.0

        # 定时循环 (2 Hz)
        self.timer = self.create_timer(0.5, self.run)
        self.get_logger().info('SentryBehavior v2 ready')
        self.get_logger().info('States: INIT → PATROL → CHASE → RETREAT → AVOID')

    # ---- 回调 ----
    def amcl_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n > 200:
            fn = int(n * 15 / 360)
            self.front_dist = min(
                min(msg.ranges[0:fn] if msg.ranges[0:fn] else [99.0]),
                min(msg.ranges[-fn:] if msg.ranges[-fn:] else [99.0])
            )

    def cmd_cb(self, msg):
        """接收手动命令 (用于调试和裁判信号模拟)"""
        cmd = msg.data.lower()
        if cmd == 'start':
            self.match_started = True
            self.state = 'PATROL'
            self.get_logger().info('=== MATCH STARTED ===')
        elif cmd == 'retreat':
            self.state = 'RETREAT'
        elif cmd == 'chase':
            self.enemy_detected = True
            self.state = 'CHASE'
        elif cmd == 'hp_low':
            self.robot_hp = 20.0
        elif cmd == 'reset':
            self.state = 'INIT'
            self.patrol_idx = 0
        elif cmd == 'pause':
            self.state = 'DONE'
        self.get_logger().info(f'Command: {cmd} → state: {self.state}')

    # ---- 导航辅助 ----
    def send_goal(self, x, y):
        g = PointStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.point.x = float(x)
        g.point.y = float(y)
        g.point.z = 0.0
        self.goal_pub.publish(g)
        self.last_goal_time = time.time()
        self.get_logger().info(f'→ Goal: ({x:.1f}, {y:.1f})')

    def dist_to(self, x, y):
        return math.hypot(x - self.rx, y - self.ry)

    def switch_state(self, new_state):
        old = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'State: {old} → {new_state}')
        self.state_pub.publish(String(data=new_state))

    # ---- 主循环 ----
    def run(self):
        # 模拟比赛时间流逝
        if self.match_started and self.remaining_time > 0:
            self.remaining_time -= 0.5

        # ---- 状态机 ----
        if self.state == 'INIT':
            if self.match_started:
                self.switch_state('PATROL')
            # 否则等待 start 命令

        elif self.state == 'PATROL':
            # 检查是否需要撤退
            if self.remaining_time < 30 or self.robot_hp < 25:
                self.switch_state('RETREAT')
                return

            # 发现敌方 → 追击
            if self.enemy_detected:
                self.switch_state('CHASE')
                return

            # 前方严重拥堵 → 标记避障 (simple_nav 自己会处理轻微避障)
            if self.front_dist < 0.3 and self.front_dist > 0:
                self.switch_state('AVOID')
                return

            # 正常巡逻: 循环走巡逻点
            tx, ty = self.patrol_pts[self.patrol_idx]
            if self.dist_to(tx, ty) < 0.5:
                # 到达当前点, 切到下一个
                self.patrol_idx = (self.patrol_idx + 1) % len(self.patrol_pts)
                self.get_logger().info(f'WP {self.patrol_idx-1} reached → WP {self.patrol_idx}')
                return

            # 每 3 秒重新发布目标
            if time.time() - self.last_goal_time > 3.0:
                self.send_goal(tx, ty)

        elif self.state == 'AVOID':
            # simple_nav 正在旋转避障, 持续发送当前目标
            tx, ty = self.patrol_pts[self.patrol_idx]
            if time.time() - self.last_goal_time > 2.0:
                self.send_goal(tx, ty)
            # 前方通畅了就回去
            if self.front_dist > 0.8 or self.front_dist == 0:
                self.get_logger().info('Path cleared, returning to PATROL')
                self.avoid_timer = 0.0
                self.switch_state('PATROL')
                return
            self.avoid_timer += 0.5
            if self.avoid_timer > 20.0:
                # 20s 还没通 → 跳过这个点
                self.get_logger().warn('Avoid timeout, skipping waypoint')
                self.patrol_idx = (self.patrol_idx + 1) % len(self.patrol_pts)
                self.avoid_timer = 0.0
                self.switch_state('PATROL')

        elif self.state == 'CHASE':
            # 前往中央区域拦截
            self.send_goal(0.0, 0.0)
            if self.enemy_detected == False:
                self.switch_state('PATROL')

        elif self.state == 'RETREAT':
            # 返回蓝方出生点 (安全区)
            safe_x, safe_y = -3.0, -5.0
            if self.dist_to(safe_x, safe_y) < 0.3:
                self.get_logger().info('Safe zone reached, defending...')
                if self.remaining_time <= 0:
                    self.switch_state('DONE')
                else:
                    # 血量恢复后重新巡逻
                    self.robot_hp = 100.0
                    self.switch_state('PATROL')
            elif time.time() - self.last_goal_time > 3.0:
                self.send_goal(safe_x, safe_y)

        elif self.state == 'DONE':
            self.get_logger().info('=== MATCH COMPLETE ===')
            self.state_pub.publish(String(data='DONE'))

        # 发布当前状态
        self.state_pub.publish(String(data=self.state))


def main():
    rclpy.init()
    rclpy.spin(SentryBehavior())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
