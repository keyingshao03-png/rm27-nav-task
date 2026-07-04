# RM27 导航组第二阶段考核 — 完整方案

## 1. 系统环境

- Ubuntu 22.04 + ROS2 Humble
- Gazebo Classic 11
- Python 3.10 + NumPy + PyYAML
- slam_toolbox, Nav2 (amcl, map_server, planner, controller, behaviors, bt_navigator)

## 2. 工程结构

```
sentry_ws/src/
├── deus_simulation/          # 第一阶段仿真包 (已有)
├── sentry_navigation/        # 第二阶段新增: Nav2 配置 + 地图 + launch
│   ├── config/
│   │   └── nav2_params.yaml  # Nav2 + AMCL + costmap 全参数
│   ├── launch/
│   │   ├── localization.launch.py   # map_server + AMCL
│   │   ├── nav_bringup.launch.py    # Nav2 导航栈
│   │   ├── slam_bringup.launch.py   # slam_toolbox 建图
│   │   └── sentry_auto.launch.py    # 一键启动
│   ├── maps/
│   │   ├── map.pgm
│   │   └── map.yaml
│   └── scripts/
│       ├── odom_tf_publisher.py     # odom→base_footprint TF
│       ├── moving_obstacle.py       # 动态障碍物
│       ├── simple_nav.py            # 简易导航器(降级方案)
│       └── save_map.py              # 保存地图
└── sentry_behavior/          # 第二阶段新增: 决策树
    ├── launch/
    │   └── sentry_competition.launch.py
    └── scripts/
        ├── sentry_behavior_nav2.py  # Nav2 正式状态机
        └── sentry_behavior.py       # simple_nav 降级状态机
```

### 实际文件清单

```
RM27_提交/
├── sentry_navigation/                # 导航功能包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/nav2_params.yaml       # Nav2+AMCL+costmap 全参数
│   ├── launch/
│   │   ├── localization.launch.py    # map_server + AMCL
│   │   ├── nav_bringup.launch.py     # Nav2 导航栈
│   │   ├── sentry_auto.launch.py     # 一键启动
│   │   └── slam_bringup.launch.py    # slam_toolbox 建图
│   ├── maps/
│   │   ├── map.pgm                   # 2D 栅格地图 (161×241)
│   │   └── map.yaml                  # 地图元数据
│   └── scripts/
│       ├── simple_nav.py             # 自研导航器 (降级方案)
│       ├── moving_obstacle.py        # 动态障碍物
│       ├── odom_tf_publisher.py      # TF 发布
│       ├── save_map.py               # 地图保存
│       └── check_nav2_status.sh      # 诊断脚本
├── sentry_behavior/                  # 决策功能包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/sentry_competition.launch.py
│   └── scripts/
│       ├── sentry_behavior_nav2.py   # Nav2 正式状态机
│       └── sentry_behavior.py        # 降级状态机
├── scripts/
│   ├── run_demo.sh                   # Nav2 一键启动
│   ├── demo_auto.sh                  # 自动演示
│   └── start_sentry.sh               # 仿真启动
├── media/
│   └── 演示视频
├── docs/
│   ├── README.md
│   └── 导航组考核报告.pdf
```

## 3. 安装

```bash
# 安装依赖
sudo apt install -y ros-humble-nav2-bringup ros-humble-slam-toolbox

# 复制功能包到工作空间
cp -r sentry_navigation ~/Navigation-Recruitment-Task-1/sentry_ws/src/
cp -r sentry_behavior ~/Navigation-Recruitment-Task-1/sentry_ws/src/

# 地图已安装在功能包内，默认会使用 sentry_navigation/maps/map.yaml

# 编译
cd ~/Navigation-Recruitment-Task-1/sentry_ws
colcon build --symlink-install
source install/setup.bash
```

## 4. 使用流程

### 4.1 建图 (任务2)

```bash
# 终端1: 启动仿真 + spawn 机器人 (见 README.md 原有步骤)
# 终端2: 启动建图
ros2 launch sentry_navigation slam_bringup.launch.py

# 终端3: 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

# 走完场地后保存
python3 ~/sentry_ws/src/sentry_navigation/scripts/save_map.py
```

### 4.2 定位 + 导航 (任务3+4)

```bash
# 终端1: 启动仿真 + spawn 机器人
# 终端2: 启动定位
ros2 launch sentry_navigation localization.launch.py

# 终端3: 启动导航
ros2 launch sentry_navigation nav_bringup.launch.py

# 然后在 RViz2 中用 "2D Goal Pose" 发目标点
```

### 4.3 完整比赛 (任务6)

```bash
# 一键启动 (需要仿真已在运行)
ros2 launch sentry_navigation sentry_auto.launch.py

# 手动控制比赛流程:
ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: start}"
ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: retreat}"
ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: chase}"
ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: hp_low}"
```

### 4.4 WSL 无 GUI 一键启动

```bash
bash start_sentry.sh
```

## 5. 两大导航方案

### 方案A: Nav2 完整栈（正式演示路线）

```
map_server → AMCL → global_costmap → planner_server
                                         ↓
local_costmap ← /scan → controller_server → /cmd_vel
```

**优点**: 全局规划 + 局部避障 + 恢复行为 + 行为树
**启动**: 用 `sentry_auto.launch.py`
**当前状态**: 正式演示链路采用 Nav2：`sentry_behavior_nav2.py → /navigate_to_pose → Nav2 → /cmd_vel`。`simple_nav.py` 仅作为降级备份，不在 `sentry_auto.launch.py` 中启动。

### 方案B: 自研 simple_nav（备份降级方案）

```
AMCL(/amcl_pose) + /scan → simple_nav → /cmd_vel
```

**优点**: 无 Nav2 lifecycle 依赖，WSL 环境下稳定；目标点由 `/goal_point` 发布，节点根据 `/amcl_pose` 或 `/odom` 估计位置，并结合 `/scan` 做反应式避障。
**启动**: `ros2 run sentry_navigation simple_nav.py`

## 6. 状态机

```
INIT → PATROL → CHASE → RETREAT → AVOID → DONE
         ↑         ↑         ↑
         └─────────┴─────────┘
           (触发条件恢复)
```

| 状态 | 触发条件 | 行为 |
|------|---------|------|
| INIT | 系统启动 | 等待 start 命令 |
| PATROL | 默认 | 循环走巡逻点 (6个) |
| CHASE | enemy_detected | 冲向中央拦截 |
| RETREAT | HP<25 或 time<30s | 返回出生点 |
| AVOID | sentry_behavior_nav2: front_dist<0.35m | 状态展示；实际避障由 Nav2 local costmap + controller 完成 |
| DONE | 时间到 | 停止并输出日志 |

### 状态转移图

```text
                    ┌─────────────────────────┐
                    │          INIT            │
                    │  等待定位 & start 命令    │
                    └────────────┬────────────┘
                                 │ start
                                 ▼
                    ┌─────────────────────────┐
              ┌────│         PATROL           │◄──────────────┐
              │    │  循环走巡逻点 (6个)       │               │
              │    └──┬──────────┬─────────┬──┘               │
              │       │          │         │                  │
              │  enemy│     front│    hp<25│                  │
              │  _det  │     _dist│    or   │                  │
              │       │      <0.3│  time<30│                  │
              │       ▼          ▼         ▼                  │
              │  ┌────────┐ ┌────────┐ ┌────────┐            │
              │  │ CHASE  │ │ AVOID  │ │RETREAT │            │
              │  │去中央   │ │等待畅通 │ │回出生点 │            │
              │  │拦截敌方 │ │或超时跳 │ │安全区   │──► DONE   │
              │  └───┬────┘ └───┬────┘ └───┬────┘  (时间到)  │
              │      │          │          │                  │
              │      │     front│    safe_zone                 │
              │ enemy│     _dist│    _reached                 │
              │ _lost│      >0.8│     → hp恢复                │
              │      │          │          │                  │
              └──────┴──────────┴──────────┘                  │
                    (所有恢复路径回到 PATROL)                   │
                    └──────────────────────────────────────────┘
```

## 7. TF 链路

```
map ──(AMCL)──→ odom ──(odom_tf)──→ base_footprint
                                       └── base_link
                                             └── gimbal_link
                                                   ├── imu_link
                                                   └── laser_link
```

## 8. 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /scan | LaserScan | 2D 激光 (360°) |
| /odom | Odometry | 里程计 |
| /amcl_pose | PoseWithCovariance | AMCL 定位 |
| /navigate_to_pose | nav2_msgs/action/NavigateToPose | Nav2 目标 action |
| /goal_point | PointStamped | simple_nav 降级方案目标 |
| /sentry_state | String | 当前状态 |
| /sentry_command | String | 手动命令 (start/retreat/chase/hp_low/pause) |
| /cmd_vel | Twist | 底盘控制 |

## 9. 常见问题

| 问题 | 解决 |
|------|------|
| Nav2 lifecycle 节点 inactive | lifecycle_manager 已配置自动激活；仍不稳定时切换到 simple_nav 降级路线 |
| AMCL 不收敛 | 检查 initial_pose 是否在出生点附近 (-3, -5)，/scan 是否有数据；演示中可降级使用 /odom |
| costmap 空白 | 检查 global_frame 是否为 "map"，obstacle_layer 是否订阅了 /scan，local_costmap 是否为 YAML 顶层节点 |
| 机器人不动 | `ros2 topic echo /cmd_vel` 看是否有输出；检查 TF 链路 |
| Gazebo 模型下载超时 | `GAZEBO_MODEL_DATABASE_URI=""` |
| WSL Gazebo GUI 崩溃 | 用 gzserver 无头模式 |

## 10. 已完成 vs 未完成

### 已完成
- [x] 仿真环境搭建 + 传感器链路检查
- [x] slam_toolbox 2D 建图 + 地图保存
- [x] AMCL / Nav2 定位与导航配置
- [x] Nav2 完整参数文件 (planner + controller + costmap + behavior)
- [x] 自研 simple_nav 降级方案
- [x] 动态障碍物生成与移动 (Gazebo 物理模型 + /set_entity_state)
- [x] 状态机决策 (5 状态自动切换)
- [x] 一键启动脚本

### 待完善
- [ ] Nav2 lifecycle 与 AMCL 在 WSL 无头环境下的稳定复现
- [ ] 重定位前后对比截图或视频
- [ ] 接入真实裁判系统 (目前手动发 /sentry_command 模拟)
- [ ] 三点游龙自主巡逻逻辑
- [ ] 自动化评分脚本
- [ ] 完整比赛流程视频录制

## 11. 关键参数速查

| 参数 | 值 | 位置 |
|------|----|------|
| 地图分辨率 | 0.05 m/pixel | slam_toolbox / nav2_params |
| 机器人半径 | 0.3 m | costmap |
| 障碍膨胀半径 | 0.55 m | costmap inflation_layer |
| 最大线速度 | 0.5 m/s | controller_server |
| AMCL 粒子数 | 500~2000 | amcl |
| 到达容忍度 | 0.25 m | goal_checker / simple_nav |
| simple_nav 前方避障 | 0.85 m | simple_nav.py |
| simple_nav 侧方避障 | 0.6 m | simple_nav.py |
| sentry_behavior AVOID 触发 | 0.3 m | sentry_behavior.py |
