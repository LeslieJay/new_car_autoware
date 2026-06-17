# Reverse Parking Controller

## 概述

倒车停车路径跟踪控制器，订阅 `reverse_parking_planner` 规划的轨迹，使用 **Pure Pursuit + PID** 算法进行路径跟踪，将控制指令发布到 `vehicle_cmd_gate` 节点的 `input/external` 话题。

## 系统架构

```
reverse_parking_planner ──(trajectory)──> reverse_parking_controller ──(control_cmd)──> vehicle_cmd_gate
                                               │                            │
                                    odometry ──┘                            ├── gear_cmd
                                                                            ├── turn_indicators_cmd
                                                                            └── hazard_lights_cmd
```

## 话题接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `~/input/trajectory` | `autoware_planning_msgs/msg/Trajectory` | 倒车规划器输出的轨迹 |
| `~/input/odometry` | `nav_msgs/msg/Odometry` | 车辆里程计 |

### 发布话题（→ vehicle_cmd_gate input/external）

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `~/output/control_cmd` | `autoware_control_msgs/msg/Control` | 转向+纵向控制指令 |
| `~/output/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | 前进/倒车档位 |
| `~/output/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | 转向灯 |
| `~/output/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | 危险警示灯 |
| `~/debug/markers` | `visualization_msgs/msg/MarkerArray` | 调试可视化 |

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `control_rate` | double | 30.0 | 控制频率 [Hz] |
| `wheel_base` | double | 1.0 | 轴距 [m] |
| `lookahead_distance` | double | 1.0 | 基础前视距离 [m] |
| `min_lookahead_distance` | double | 0.5 | 最小前视距离 [m] |
| `lookahead_ratio` | double | 2.0 | 前视距离速度比例系数 |
| `pid.kp` | double | 1.0 | PID 比例增益 |
| `pid.ki` | double | 0.1 | PID 积分增益 |
| `pid.kd` | double | 0.05 | PID 微分增益 |
| `max_acceleration` | double | 1.0 | 最大加速度 [m/s²] |
| `max_deceleration` | double | -2.0 | 最大减速度 [m/s²] |
| `goal_distance_threshold` | double | 0.3 | 终点距离判定阈值 [m] |
| `goal_yaw_threshold` | double | 0.1 | 终点航向判定阈值 [rad] |
| `max_steering_angle` | double | 0.6 | 最大转向角 [rad] |

## 控制算法

### 横向控制 - Pure Pursuit
- 根据当前速度自适应调整前视距离：`Ld = max(min_Ld, ratio * |v|)`
- 在车体坐标系下计算目标点方位角 α
- 转向角：`δ = atan(2L·sin(α) / Ld)`
- 倒车时自动反转转向角方向

### 纵向控制 - PID
- 速度误差 PID 控制
- 积分抗饱和限幅
- 加速度输出限幅

### 档位控制
- 根据轨迹点速度方向自动切换 DRIVE/REVERSE 档
- 到达终点后切换 PARK 档

## 室内 AGV 倒车充电优化总结（2026-03）

针对末端对接精度和低速稳定性，控制侧优化如下：

### 1. 倒车专用前视距离

- 新增 `reverse_lookahead_distance`，倒车时使用更短前视距离。
- 近距离对接阶段可降低横向跟踪误差。

### 2. 最终接近阶段横向误差修正

- 在 `final_approach_distance` 范围内，叠加 Stanley 风格横向误差修正。
- 修正增益由 `stanley_gain` 控制，用于增强末端纠偏能力。

### 3. 基于目标距离的速度收敛

- 新增 `calcDistanceToGoal`，按距离目标点的远近调制速度。
- 在最终接近区内，目标速度逐步下降到 `final_approach_speed` 蠕行值。

### 4. 换向安全保护

- 当检测到前进/倒车方向变化时，若车速仍高于 `stop_velocity_threshold`，
    先制动再切档，避免机械冲击。

### 5. 充电对接阈值收紧

- `goal_distance_threshold` 调整为 0.05 m。
- `goal_yaw_threshold` 调整为 0.03 rad。
- 配合低速接近策略，提升车位对接一致性。

新增参数：

- `reverse_lookahead_distance`：倒车前视距离（默认 0.4 m）
- `stanley_gain`：横向误差修正增益（默认 1.5）
- `final_approach_distance`：最终接近距离（默认 1.0 m）
- `final_approach_speed`：最终接近速度（默认 0.05 m/s）

## 启动

```bash
# 单独启动
ros2 launch reverse_parking_controller reverse_parking_controller.launch.py

# 与规划器联合启动
ros2 launch reverse_parking_planner reverse_parking_planner.launch.py &
ros2 launch reverse_parking_controller reverse_parking_controller.launch.py
```

## 编译

```bash
colcon build --symlink-install --packages-select reverse_parking_controller
```
