#!/bin/bash
# 自动演示脚本 — 录屏时跑这一条就行
echo "========================================="
echo "  RM27 哨兵导航 - 自动演示"
echo "========================================="
echo ""

# 展示 topic list
echo ">>> [任务1] Topic List:"
ros2 topic list 2>/dev/null | head -20
echo ""

# 展示 TF 树
echo ">>> [任务1] TF Tree (5s):"
timeout 5 ros2 run tf2_tools view_frames 2>/dev/null
cat /tmp/frames.yaml 2>/dev/null | head -20
echo ""

# 展示 AMCL 定位
echo ">>> [任务3] AMCL Pose:"
ros2 topic echo /amcl_pose --once 2>/dev/null | head -12
echo ""

# 启动比赛
echo ">>> [任务6] Sending START..."
ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: start}" 2>/dev/null
sleep 2

# 观察 15 秒自动巡逻
echo ">>> [任务6] Watching state transitions (15s)..."
for i in $(seq 1 30); do
    STATE=$(ros2 topic echo /sentry_state --once 2>/dev/null | grep "data:" | tail -1)
    ODOM=$(ros2 topic echo /odom --once 2>/dev/null | grep "position:" -A1 | tail -1)
    echo "  [$i] $STATE | pos$ODOM"
    sleep 0.5
done

# 触发撤退
echo ""
echo ">>> [任务6] Sending RETREAT..."
ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: retreat}" 2>/dev/null
sleep 3

# 观察撤退
echo ">>> [任务6] Retreat state:"
for i in $(seq 1 6); do
    STATE=$(ros2 topic echo /sentry_state --once 2>/dev/null | grep "data:" | tail -1)
    ODOM=$(ros2 topic echo /odom --once 2>/dev/null | grep "position:" -A1 | tail -1)
    echo "  [$i] $STATE | pos$ODOM"
    sleep 0.5
done

# CMD_VEL 展示
echo ""
echo ">>> [任务5] /cmd_vel (obstacle avoidance working):"
ros2 topic echo /cmd_vel --once 2>/dev/null | head -8

echo ""
echo "========================================="
echo "  Demo complete! Stop recording."
echo "========================================="
