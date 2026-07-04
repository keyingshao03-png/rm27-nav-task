#!/usr/bin/env bash
# RM27 Nav2 demo launcher (navigation stack only).
# Use this after Gazebo + robot have already been started.
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/Navigation-Recruitment-Task-1/sentry_ws}"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

MAP_FILE="${MAP_FILE:-$(ros2 pkg prefix sentry_navigation)/share/sentry_navigation/maps/map.yaml}"

echo "========================================="
echo "  RM27 Nav2 Demo"
echo "========================================="
echo "Workspace: $WS_DIR"
echo "Map:       $MAP_FILE"
echo ""

echo "[1/3] Killing old simple_nav / old behavior nodes..."
pkill -f "simple_nav" 2>/dev/null || true
pkill -f "sentry_behavior.py" 2>/dev/null || true
pkill -f "sentry_behavior_nav2.py" 2>/dev/null || true
pkill -f "moving_obstacle.py" 2>/dev/null || true
sleep 1

echo "[2/3] Launching AMCL + Nav2 + behavior_nav2 + moving_obstacle..."
echo "      Press Ctrl+C to stop."
ros2 launch sentry_navigation sentry_auto.launch.py map_file:="$MAP_FILE" &
LAUNCH_PID=$!

sleep 8

echo "[3/3] Quick check:"
ros2 node list 2>/dev/null | sed 's/^/  /'
echo ""
if ros2 node list 2>/dev/null | grep -q "simple_nav"; then
  echo "WARN: simple_nav is running. Kill it: pkill -f simple_nav"
else
  echo "OK: simple_nav is not running."
fi
if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
  echo "OK: /navigate_to_pose action exists."
else
  echo "WARN: /navigate_to_pose action not found yet. Wait or check Nav2 lifecycle."
fi

echo ""
echo "Start patrol with:"
echo "  ros2 topic pub --once /sentry_command std_msgs/msg/String \"{data: start}\""
echo ""
wait "$LAUNCH_PID"
