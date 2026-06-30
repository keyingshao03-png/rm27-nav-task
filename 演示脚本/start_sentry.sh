#!/bin/bash
# ==============================================================
#  哨兵导航系统 — 一键启动脚本
#  RM27 导航组第二阶段考核
# ==============================================================
set -e

WS_DIR="$HOME/Navigation-Recruitment-Task-1/sentry_ws"
MAP_FILE="$HOME/map.yaml"

echo "========================================="
echo "  哨兵导航系统 一键启动"
echo "========================================="

# ---- 1. 环境变量 ----
echo "[1/5] Setting up environment..."
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=$WS_DIR/src/DEUS_simulation/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$WS_DIR/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source /opt/ros/humble/setup.bash
source $WS_DIR/install/setup.bash

# ---- 2. 清理残留进程 ----
echo "[2/5] Cleaning up..."
killall -9 gzserver gzclient gazebo 2>/dev/null || true
pkill -f "ros2" 2>/dev/null || true
sleep 1

# ---- 3. 启动 Gazebo 仿真 ----
echo "[3/5] Starting Gazebo simulation..."
gzserver --verbose \
  -slibgazebo_ros_init.so \
  -slibgazebo_ros_factory.so \
  -slibgazebo_ros_force_system.so \
  $WS_DIR/install/deus_simulation/share/deus_simulation/worlds/3v3world.sdf &
GAZEBO_PID=$!

# 等 Gazebo 起来
echo "  Waiting for Gazebo..."
sleep 5

# ---- 4. 生成机器人 ----
echo "[4/5] Spawning robot..."
# robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(ros2 run xacro xacro $WS_DIR/src/DEUS_simulation/xacro/box_lidar.xacro)" \
  -p use_sim_time:=true &
RSP_PID=$!

sleep 3

# spawn
ros2 run gazebo_ros spawn_entity.py -topic robot_description \
  -entity sentry_bot -x -3.0 -y -5.0 -z 0.15 --ros-args -p use_sim_time:=true

sleep 3

# ---- 5. 启动完整导航系统 ----
echo "[5/5] Launching navigation system..."
ros2 launch sentry_navigation sentry_auto.launch.py map_file:=$MAP_FILE

# ---- 清理 ----
echo "Shutting down..."
kill $GAZEBO_PID $RSP_PID 2>/dev/null
echo "Done."