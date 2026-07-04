#!/usr/bin/env python3
"""
动态障碍物生成器 v3 — 使用 Gazebo /set_entity_state 直接移动
"""
import rclpy
import math
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState, DeleteEntity
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist


SDF_BOX = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="moving_box">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia><ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.1</iyy><iyz>0</iyz><izz>0.1</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
        <material><ambient>1 0.2 0.2 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>'''


class MovingObstacle(Node):
    def __init__(self):
        super().__init__('moving_obstacle')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.set_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.t = 0.0
        self.spawned = False
        self.spawn_requested = False

        self.get_logger().info('Preparing moving_box obstacle...')
        self.start_timer = self.create_timer(0.5, self.try_start)

    def try_start(self):
        if self.spawn_requested:
            return
        missing = []
        if not self.spawn_cli.wait_for_service(timeout_sec=0.05):
            missing.append('/spawn_entity')
        if not self.delete_cli.wait_for_service(timeout_sec=0.05):
            missing.append('/delete_entity')
        if not self.set_state_cli.wait_for_service(timeout_sec=0.05):
            missing.append('/set_entity_state')

        if missing:
            self.get_logger().info(f'Waiting for Gazebo services: {", ".join(missing)}')
            return

        self.spawn_requested = True
        self.start_timer.cancel()

        req = DeleteEntity.Request()
        req.name = 'moving_box'
        self.delete_cli.call_async(req)

        self._spawn_requested = True
        self.spawn()

    def spawn(self):
        req = SpawnEntity.Request()
        req.name = 'moving_box'
        req.xml = SDF_BOX
        req.initial_pose = Pose(position=Point(x=-1.2, y=-2.2, z=0.5))
        future = self.spawn_cli.call_async(req)
        future.add_done_callback(self.spawn_done)

        # 定时直接更新 Gazebo 实体位姿，确保盒子真实运动。
        self.timer = self.create_timer(0.1, self.move)

    def spawn_done(self, future):
        try:
            result = future.result()
            self.spawned = bool(result.success)
            if result.success:
                self.get_logger().info('Box spawned. Moving in circular pattern...')
            else:
                self.get_logger().warn(f'Failed to spawn moving_box: {result.status_message}')
        except Exception as exc:
            self.get_logger().error(f'Spawn service failed: {exc}')

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def move(self):
        """让盒子在通道附近做圆周运动，触发导航避障。"""
        if not self.spawned:
            return

        self.t += 0.1
        omega = 0.6

        state = EntityState()
        state.name = 'moving_box'
        state.reference_frame = 'world'
        # 横穿巡逻路线 (-1.8,-3.0) -> (-1.2,-2.0)
        state.pose.position.x = -1.4 + 0.8 * math.sin(self.t * omega)
        state.pose.position.y = -2.4
        yaw = 0.0
        state.pose.position.z = 0.5
        state.pose.orientation = self.yaw_to_quaternion(yaw)
        state.twist = Twist()
        state.twist.linear.x = -0.8 * omega * math.sin(self.t * omega)
        state.twist.angular.z = omega

        req = SetEntityState.Request()
        req.state = state
        self.set_state_cli.call_async(req)


def main():
    rclpy.init()
    node = MovingObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
