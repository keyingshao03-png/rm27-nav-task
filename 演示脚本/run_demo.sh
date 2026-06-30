#!/bin/bash
echo "[1/4] Killing old processes..."
pkill -f simple_nav 2>/dev/null
pkill -f sentry_behavior 2>/dev/null
pkill -f moving_obstacle 2>/dev/null
sleep 2

echo "[2/4] Copying latest code..."
SRC=/mnt/c/Users/Lenovo/OneDrive/Desktop/nav_task
DST=~/Navigation-Recruitment-Task-1/sentry_ws/src
cp $SRC/sentry_behavior/scripts/sentry_behavior.py $DST/sentry_behavior/scripts/
cp $SRC/sentry_navigation/scripts/moving_obstacle.py $DST/sentry_navigation/scripts/
cp $SRC/sentry_navigation/scripts/simple_nav.py $DST/sentry_navigation/scripts/

echo "[3/4] Starting nodes..."
ros2 run sentry_navigation simple_nav.py &
sleep 2
ros2 run sentry_behavior sentry_behavior.py &
sleep 2
ros2 run sentry_navigation moving_obstacle.py &
sleep 2

echo "[4/4] Checking..."
ROS_NODES=$(ros2 node list 2>/dev/null)
echo "$ROS_NODES"

for n in simple_nav sentry_behavior moving_obstacle amcl map_server; do
    if echo "$ROS_NODES" | grep -q "$n"; then
        echo "  OK: $n"
    else
        echo "  MISS: $n"
    fi
done

echo ""
echo "=== Ready! Start: ==="
echo "  ros2 topic pub --once /sentry_command std_msgs/msg/String \"{data: start}\""
echo "=== Watch: ==="
echo "  ros2 topic echo /sentry_state"
