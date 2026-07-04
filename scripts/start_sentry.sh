#!/usr/bin/env bash
# One-key launcher: Gazebo + sentry robot + Nav2 navigation.
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/Navigation-Recruitment-Task-1/sentry_ws}"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

WORLD="$WS_DIR/install/deus_simulation/share/deus_simulation/worlds/3v3world.sdf"
ROBOT_XACRO="$WS_DIR/src/DEUS_simulation/xacro/box_lidar.xacro"
MAP_FILE="${MAP_FILE:-$(ros2 pkg prefix sentry_navigation)/share/sentry_navigation/maps/map.yaml}"

export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="$WS_DIR/src/DEUS_simulation/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$WS_DIR/install/ros2_livox_simulation/lib:${GAZEBO_PLUGIN_PATH:-}"

echo "========================================="
echo "  Sentry Navigation One-key Launcher"
echo "========================================="
echo "Workspace: $WS_DIR"
echo "Map:       $MAP_FILE"
echo ""

echo "[1/5] Cleaning old processes..."
killall -9 gzserver gzclient gazebo 2>/dev/null || true
pkill -f "simple_nav" 2>/dev/null || true
pkill -f "sentry_behavior.py" 2>/dev/null || true
pkill -f "sentry_behavior_nav2.py" 2>/dev/null || true
pkill -f "moving_obstacle.py" 2>/dev/null || true
pkill -f "ros2 launch sentry_navigation" 2>/dev/null || true
sleep 1

echo "[2/5] Starting Gazebo..."
gzserver --verbose \
  -slibgazebo_ros_init.so \
  -slibgazebo_ros_factory.so \
  -slibgazebo_ros_force_system.so \
  "$WORLD" &
GAZEBO_PID=$!
sleep 5

echo "[3/5] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(ros2 run xacro xacro "$ROBOT_XACRO")" \
  -p use_sim_time:=true &
RSP_PID=$!
sleep 3

echo "[4/5] Spawning robot..."
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity sentry_bot \
  -x -3.0 -y -5.0 -z 0.15 \
  --ros-args -p use_sim_time:=true
sleep 3

echo "[5/5] Launching Nav2 stack..."
ros2 launch sentry_navigation sentry_auto.launch.py map_file:="$MAP_FILE"

kill "$GAZEBO_PID" "$RSP_PID" 2>/dev/null || true
