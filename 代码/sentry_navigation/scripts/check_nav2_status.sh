#!/usr/bin/env bash
echo "========================================="
echo "  哨兵导航系统 一键诊断"
echo "========================================="
echo ""

echo "[1] Nodes running:"
ros2 node list 2>/dev/null
echo ""

echo "[2] Key topics:"
for t in /odom /scan /cmd_vel /amcl_pose /goal_point /sentry_state /tf /tf_static /map; do
    if ros2 topic list 2>/dev/null | grep -q "$t"; then
        echo "  OK  $t"
    else
        echo "  --  $t (not found)"
    fi
done
echo ""

echo "[3] AMCL pose:"
ros2 topic echo /amcl_pose --once 2>&1 | head -8
echo ""

echo "[4] Odometry:"
ros2 topic echo /odom --once 2>&1 | head -8
echo ""

echo "[5] /cmd_vel (last):"
ros2 topic echo /cmd_vel --once 2>&1 | head -8
echo ""

echo "[6] TF map->base_footprint:"
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint --once 2>&1 | head -5
echo ""

echo "========================================="
echo "  Done. Scroll up to see results."
echo "========================================="
