# 任务1：仿真环境与传感器链路检查

## 1. ros2 topic list

```
/clock
/cmd_vel
/imu
/joint_states
/livox/lidar
/livox/lidar_PointCloud2
/odom
/parameter_events
/performance_metrics
/robot_description
/rosout
/set_gimbal_angle
/tf
/tf_static
```

## 2. TF 坐标系树

```
base_footprint
  └── base_link
        └── gimbal_link (TF: 60.985 Hz)
              ├── imu_link (static)
              └── mid360 (static, LiDAR frame)
```

TF 数据源：
- `robot_state_publisher`：发布 base_link → gimbal_link → imu_link → mid360
- `planar_move`：发布 `/odom` 话题，publish_odom_tf 设为 false
- 缺失：`odom → base_footprint` TF（需在后续任务中补上，由 Point-LIO 或 robot_localization 发布）

## 3. 传感器与数据链路

| 数据源 | 话题/节点 | 状态 |
|--------|----------|------|
| 里程计 | /odom (planar_move) | ✅ |
| LiDAR 点云 | /livox/lidar_PointCloud2 (mid360_plugin) | ✅ |
| IMU | /imu (imu_plugin) | ✅ |
| 底盘控制 | /cmd_vel → planar_move | ✅ |
| Joint States | /joint_states | ✅ |
| TF 发布 | /tf, /tf_static (robot_state_publisher) | ✅ |

## 4. 运行节点

```
/gazebo
/gazebo_ros_joint_pose_trajectory
/gazebo_ros_joint_state_publisher
/imu_plugin
/mid360_plugin
/planar_move
/robot_state_publisher
/sentry_control
```

## 5. 环境配置注意事项

启动前需设置 Gazebo 插件路径：
```bash
export GAZEBO_PLUGIN_PATH=~/Navigation-Recruitment-Task-1/sentry_ws/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=~/Navigation-Recruitment-Task-1/sentry_ws/install/deus_simulation/share:$GAZEBO_MODEL_PATH
```

## 6. 结论

第一阶段仿真环境核心链路正常，传感器数据（LiDAR、IMU、里程计）和底盘控制链路已验证通过。`odom → base_footprint` TF 未发布，后续导航任务中需通过 Point-LIO 或 slam_toolbox/robot_localization 补充。
