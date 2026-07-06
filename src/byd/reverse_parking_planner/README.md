# Reverse Parking Planner

AGV 室内倒车停车**统一节点**：直线倒车轨迹规划 + Pure Pursuit/PID 路径跟踪。

## 功能

- 通过服务设置目标位姿并生成直线倒车轨迹
- 自动跟踪并输出控制指令
- 可选发布轨迹与 RViz 可视化（调试用）

## 架构

```
Odometry + SetGoalPose/Trigger
            │
            ▼
   reverse_parking_planner (单节点)
     ├─ 直线倒车规划
     ├─ 内存轨迹缓冲
     └─ Pure Pursuit + PID 控制
            │
            ├─ control_cmd / gear_cmd / indicators
            └─ trajectory / markers (可选观测)
```

## 接口

### 订阅

| Topic | Type | Description |
|-------|------|-------------|
| `~/input/odometry` | `nav_msgs/Odometry` | 当前车辆位姿 |

### 发布

| Topic | Type | Description |
|-------|------|-------------|
| `~/output/trajectory` | `autoware_planning_msgs/Trajectory` | 规划轨迹（调试/观测） |
| `~/output/path_markers` | `visualization_msgs/MarkerArray` | 路径可视化 |
| `~/output/control_cmd` | `autoware_control_msgs/Control` | 控制指令 |
| `~/output/gear_cmd` | `autoware_vehicle_msgs/GearCommand` | 档位指令 |
| `~/output/turn_indicators_cmd` | `TurnIndicatorsCommand` | 转向灯 |
| `~/output/hazard_lights_cmd` | `HazardLightsCommand` | 危险警示灯 |

### 服务

| Service | Type | Description |
|---------|------|-------------|
| `~/set_goal_pose` | `reverse_parking_planner/srv/SetGoalPose` | 设置目标并规划 |
| `~/trigger_planning` | `std_srvs/Trigger` | 使用已有目标重新规划 |

## 规划约束（直线倒车）

仅在以下条件满足时生成路径，否则拒绝规划：

- 目标在车辆后方（沿目标航向的纵向投影 > 0）
- 横向偏差 ≤ `max_straight_lateral_error`
- 航向偏差 ≤ `max_straight_yaw_error`
- 倒车距离 ≥ `min_reverse_distance`

## 使用方法

### 编译

```bash
colcon build --packages-select reverse_parking_planner
source install/setup.bash
```

### 启动（推荐通过 byd_launch）

```bash
ros2 launch byd_launch reverse_parking.launch.py
```

或单独启动：

```bash
ros2 launch reverse_parking_planner reverse_parking_planner.launch.py
```

### 设置目标并规划

```bash
ros2 service call /reverse_parking_planner/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

### 重新规划

```bash
ros2 service call /reverse_parking_planner/trigger_planning std_srvs/srv/Trigger
```

## 说明

- 原 `reverse_parking_controller` 包已移除，控制逻辑已合并到本节点。
- 轨迹 `longitudinal_velocity_mps < 0` 表示倒车段；终点速度为 0。
