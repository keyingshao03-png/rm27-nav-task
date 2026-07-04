#!/usr/bin/env bash
echo "========================================="
echo "  Sentry Nav2 Diagnostic"
echo "========================================="
echo ""

echo "[1] Nodes running:"
ros2 node list 2>/dev/null | sort

echo ""
echo "[2] Nav2 action:"
if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
  echo "  OK  /navigate_to_pose"
else
  echo "  MISS /navigate_to_pose"
fi

echo ""
echo "[3] Old simple_nav check:"
if ros2 node list 2>/dev/null | grep -q "simple_nav"; then
  echo "  WARN simple_nav is running. It may抢 /cmd_vel. Run: pkill -f simple_nav"
else
  echo "  OK  simple_nav is not running"
fi

echo ""
echo "[4] Key topics:"
for t in /odom /scan /cmd_vel /amcl_pose /sentry_state /tf /tf_static /map; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    echo "  OK  $t"
  else
    echo "  --  $t (not found)"
  fi
done

echo ""
echo "[5] Lifecycle states:"
for n in /map_server /amcl /planner_server /controller_server /behavior_server /bt_navigator /velocity_smoother; do
  echo -n "  $n: "
  ros2 lifecycle get "$n" 2>/dev/null || echo "not available"
done

echo ""
echo "[6] AMCL pose sample:"
ros2 topic echo /amcl_pose --once 2>&1 | head -10

echo ""
echo "[7] /cmd_vel sample:"
ros2 topic echo /cmd_vel --once 2>&1 | head -10

echo ""
echo "========================================="
echo "Done."
echo "========================================="
