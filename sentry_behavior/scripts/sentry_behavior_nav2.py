#!/usr/bin/env python3
"""
哨兵比赛行为决策 Nav2 版
=======================
核心变化：
- 不再向 /goal_point 发布目标给 simple_nav
- 直接调用 Nav2 的 /navigate_to_pose Action
- 由 Nav2 完成全局规划、局部规划、动态避障和恢复行为
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SentryBehaviorNav2(Node):
    def __init__(self):
        super().__init__('sentry_behavior_nav2')

        self.state_pub = self.create_publisher(String, '/sentry_state', 10)
        self.create_subscription(String, '/sentry_command', self.cmd_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 比赛状态
        self.state = 'INIT'
        self.match_started = False
        self.enemy_detected = False
        self.robot_hp = 100.0
        self.remaining_time = 300.0

        # 当前位姿与障碍距离
        self.rx = 0.0
        self.ry = 0.0
        self.has_pose = False
        self.front_dist = 99.0

        # 巡逻点：尽量选走廊中线，避免目标点连线穿墙
        self.patrol_pts = [
            (-3.0, -5.0),
            (-2.4, -4.0),
            (-1.8, -3.0),
            (-1.2, -2.0),
            (-0.6, -1.2),
            (0.0, 0.0),
        ]
        self.patrol_idx = 0

        self.goal_handle = None
        self.goal_active = False
        self.active_goal = None
        self.goal_seq = 0  # 用于忽略旧 goal 的回调，避免新旧目标互相干扰
        self.last_goal_time = 0.0
        self.last_retry_time = 0.0
        self.state_start_time = time.time()

        self.timer = self.create_timer(0.5, self.run)
        self.get_logger().info('SentryBehaviorNav2 ready. Waiting for /sentry_command start')

    def amcl_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        self.has_pose = True

    def scan_cb(self, msg):
        def sector_min(center_angle, half_width):
            vals = []
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r):
                    continue
                if r < msg.range_min or r > msg.range_max:
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                diff = math.atan2(
                    math.sin(angle - center_angle),
                    math.cos(angle - center_angle)
                )
                if abs(diff) <= half_width:
                    vals.append(r)
            return min(vals) if vals else 99.0

        self.front_dist = sector_min(0.0, math.radians(30))

    def cmd_cb(self, msg):
        cmd = msg.data.lower().strip()
        if cmd == 'start':
            self.match_started = True
            self.switch_state('PATROL')
        elif cmd == 'retreat':
            # 切换任务前先取消旧目标，避免旧 goal result 覆盖新状态
            self.cancel_current_goal()
            self.switch_state('RETREAT')
        elif cmd == 'chase':
            self.enemy_detected = True
            self.cancel_current_goal()
            self.switch_state('CHASE')
        elif cmd == 'hp_low':
            self.robot_hp = 20.0
        elif cmd == 'reset':
            self.cancel_current_goal()
            self.match_started = False
            self.enemy_detected = False
            self.robot_hp = 100.0
            self.remaining_time = 300.0
            self.patrol_idx = 0
            self.switch_state('INIT')
        elif cmd == 'pause':
            self.cancel_current_goal()
            self.switch_state('DONE')
        self.get_logger().info(f'Command: {cmd} -> {self.state}')

    def switch_state(self, new_state):
        if self.state != new_state:
            old = self.state
            self.state = new_state
            self.state_start_time = time.time()
            self.get_logger().info(f'State: {old} -> {new_state}')
        self.state_pub.publish(String(data=self.state))

    def dist_to(self, x, y):
        return math.hypot(x - self.rx, y - self.ry)

    def make_goal(self, x, y, yaw=0.0):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def cancel_current_goal(self):
        """取消当前 Nav2 目标，并让后续旧回调失效。"""
        self.goal_seq += 1
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'Cancel goal failed: {exc}')
        self.goal_handle = None
        self.goal_active = False
        self.active_goal = None

    def send_nav2_goal(self, x, y, yaw=0.0):
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            if time.time() - self.last_retry_time > 2.0:
                self.get_logger().warn('Waiting for Nav2 action server /navigate_to_pose...')
                self.last_retry_time = time.time()
            return

        # 同一个目标已经在执行，不要反复发送，否则会导致 Nav2 不断取消重发
        if self.goal_active and self.active_goal is not None:
            if math.hypot(self.active_goal[0] - x, self.active_goal[1] - y) < 0.05:
                return

        self.active_goal = (float(x), float(y))
        self.goal_active = True
        self.last_goal_time = time.time()
        self.goal_seq += 1
        seq = self.goal_seq

        goal_msg = self.make_goal(x, y, yaw)
        self.get_logger().info(f'Nav2 goal -> ({x:.2f}, {y:.2f})')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_cb(f, seq))

    def goal_response_cb(self, future, seq):
        if seq != self.goal_seq:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self.goal_active = False
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.goal_result_cb(f, seq))

    def goal_result_cb(self, future, seq):
        if seq != self.goal_seq:
            return
        result = future.result()
        status = result.status
        self.goal_active = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded')
            if self.state == 'PATROL':
                self.patrol_idx = (self.patrol_idx + 1) % len(self.patrol_pts)
                self.active_goal = None
            elif self.state == 'RETREAT':
                self.robot_hp = 100.0
                self.switch_state('PATROL')
            elif self.state == 'CHASE':
                self.enemy_detected = False
                self.switch_state('PATROL')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Nav2 goal canceled')
            self.active_goal = None
        else:
            self.get_logger().warn(f'Nav2 goal failed, status={status}; will retry')
            self.active_goal = None

    def run(self):
        if self.match_started and self.remaining_time > 0:
            self.remaining_time -= 0.5

        if self.state == 'INIT':
            self.state_pub.publish(String(data='INIT'))
            return

        if not self.has_pose:
            self.get_logger().warn('Waiting for /amcl_pose...')
            self.state_pub.publish(String(data=self.state))
            return

        if self.state == 'PATROL':
            if self.remaining_time < 30 or self.robot_hp < 25:
                self.goal_active = False
                self.switch_state('RETREAT')
                return

            if self.enemy_detected:
                self.goal_active = False
                self.switch_state('CHASE')
                return

            # 这里不直接控制速度，只用于状态展示；真正避障由 Nav2 local costmap + controller 完成
            if 0.0 < self.front_dist < 0.35:
                self.switch_state('AVOID')
                return

            tx, ty = self.patrol_pts[self.patrol_idx]
            if self.dist_to(tx, ty) < 0.35 and not self.goal_active:
                self.patrol_idx = (self.patrol_idx + 1) % len(self.patrol_pts)
                tx, ty = self.patrol_pts[self.patrol_idx]

            self.send_nav2_goal(tx, ty)

        elif self.state == 'AVOID':
            # 不发布 /cmd_vel，不抢 Nav2。障碍物消失后回到 PATROL。
            if self.front_dist > 0.8 or self.front_dist == 99.0:
                self.switch_state('PATROL')
            elif time.time() - self.state_start_time > 8.0:
                # 长时间堵塞时交给 Nav2 恢复，同时状态回巡逻，避免卡在 AVOID
                self.switch_state('PATROL')

        elif self.state == 'CHASE':
            self.send_nav2_goal(0.0, 0.0)

        elif self.state == 'RETREAT':
            self.send_nav2_goal(-3.0, -5.0)

        elif self.state == 'DONE':
            self.state_pub.publish(String(data='DONE'))
            return

        self.state_pub.publish(String(data=self.state))


def main():
    rclpy.init()
    node = SentryBehaviorNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
