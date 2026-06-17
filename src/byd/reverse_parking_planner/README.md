# Reverse Parking Planner

AGV室内倒车停车规划模块，使用 **Reeds-Shepp 曲线** 规划从当前位姿到目标停车点的路径。

## 功能特点

- ✅ 支持前进和倒车混合路径规划
- ✅ 使用 Reeds-Shepp 曲线计算最优路径
- ✅ 自动识别路径段的行驶方向
- ✅ 输出符合 Autoware 标准的 Trajectory 消息
- ✅ RViz 可视化（前进绿色，倒车红色）

## 算法原理

### Reeds-Shepp 曲线

Reeds-Shepp 曲线是一种用于非完整约束车辆（如汽车）的最短路径算法。与 Dubins 曲线不同，它同时考虑**前进**和**后退**运动。

路径由以下基本动作组成：
- **L+**: 左转前进
- **L-**: 左转后退
- **R+**: 右转前进
- **R-**: 右转后退
- **S+**: 直行前进
- **S-**: 直行后退

算法会在 **18 种路径类型** 中搜索最短的组合。

### 路径类型示例

```
CSC: L+S+L+, L+S+R+, R+S+L+, R+S+R+
CCC: L+R-L-, R+L-R-
CCCC: L+R+L-R-, L+R-L-R+
CCSC: L+R-S-L-, L+R-S-R-
CCSCC: L+R-S-L-R+
```

## 接口

### 输入

| Topic | Type | Description |
|-------|------|-------------|
| `~/input/odometry` | `nav_msgs/Odometry` | 当前车辆位姿 |
| `~/input/goal_pose` | `geometry_msgs/PoseStamped` | 目标停车位姿 |

### 输出

| Topic | Type | Description |
|-------|------|-------------|
| `~/output/trajectory` | `autoware_planning_msgs/Trajectory` | 规划的轨迹（含速度方向） |
| `~/output/path_markers` | `visualization_msgs/MarkerArray` | 可视化标记 |

### 服务

| Service | Type | Description |
|---------|------|-------------|
| `~/trigger_planning` | `std_srvs/Trigger` | 手动触发路径规划 |

## 参数

```yaml
# 车辆参数
wheel_base: 1.0           # 轴距 [m]
min_turning_radius: 2.0   # 最小转弯半径 [m]
vehicle_length: 1.8       # 车长 [m]
vehicle_width: 1.2        # 车宽 [m]

# 路径参数
path_resolution: 0.1      # 路径采样分辨率 [m]
velocity_forward: 0.5     # 前进速度 [m/s]
velocity_reverse: -0.3    # 倒车速度 [m/s] (负值)

# 目标停车点
goal_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0

# 发布频率
publish_rate: 10.0        # Hz
```

## 使用方法

### 1. 编译

```bash
cd ~/autoware
colcon build --packages-select reverse_parking_planner
source install/setup.bash
```

### 2. 启动

```bash
ros2 launch reverse_parking_planner reverse_parking_planner.launch.py
```

### 3. 发送目标点

```bash
# 通过 topic 发送目标位姿
ros2 topic pub /planning/mission_planning/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"

# 或通过服务触发规划
ros2 service call /reverse_parking_planner/trigger_planning std_srvs/srv/Trigger
```

### 4. 可视化

在 RViz 中添加 MarkerArray 显示：
- Topic: `/planning/parking/path_markers`
- 绿色：前进路径段
- 红色：倒车路径段

## 输出轨迹格式

轨迹点的 `longitudinal_velocity_mps` 字段：
- **正值**: 前进
- **负值**: 倒车

控制器会根据速度符号自动切换档位。

## 与 Autoware 控制器集成

本模块输出的轨迹可直接被以下控制器处理：

1. **PID 纵向控制器**: 根据 `longitudinal_velocity_mps` 符号判断 `Shift::Forward` 或 `Shift::Reverse`
2. **Pure Pursuit 横向控制器**: 根据速度符号调整前视点搜索方向

## 架构图

```
                    ┌─────────────────────────────┐
                    │  reverse_parking_planner    │
                    └─────────────┬───────────────┘
                                  │
        ┌─────────────────────────┼─────────────────────────┐
        │                         │                         │
        ▼                         ▼                         ▼
┌───────────────┐        ┌───────────────┐        ┌───────────────┐
│   Odometry    │        │   Goal Pose   │        │   Trigger     │
│   Subscriber  │        │   Subscriber  │        │   Service     │
└───────┬───────┘        └───────┬───────┘        └───────┬───────┘
        │                         │                         │
        └─────────────────────────┼─────────────────────────┘
                                  │
                                  ▼
                    ┌─────────────────────────────┐
                    │   Reeds-Shepp Planner       │
                    │   (计算最优路径)             │
                    └─────────────┬───────────────┘
                                  │
                                  ▼
                    ┌─────────────────────────────┐
                    │   Path Sampling             │
                    │   (采样路径点+方向标记)      │
                    └─────────────┬───────────────┘
                                  │
                    ┌─────────────┴───────────────┐
                    │                             │
                    ▼                             ▼
        ┌─────────────────────┐       ┌─────────────────────┐
        │  Trajectory         │       │  Visualization      │
        │  Publisher          │       │  Publisher          │
        └─────────────────────┘       └─────────────────────┘
```

## 参考

- Reeds, J.A. and Shepp, L.A., "Optimal paths for a car that goes both forwards and backwards", Pacific Journal of Mathematics, 1990
- [OMPL Reeds-Shepp Implementation](https://ompl.kavrakilab.org/)

## 室内 AGV 倒车充电优化总结（2026-03）

针对室内 AGV 倒车充电对接场景，规划侧做了以下优化：

### 1. 两阶段路径生成（预接近点 + 最终直线倒车）

- 先用 Reeds-Shepp 规划到预接近点。
- 再拼接固定长度的直线倒车段进入目标位姿。
- 这样可保证最后一段曲率为 0，提升充电插头对准精度。

新增参数：

- `final_approach_distance`：最终直线倒车接近距离，默认 1.0 m。

### 2. 速度剖面优化（终点减速 + 换向减速 + 蠕行）

- 在终点前 `decel_distance` 范围内平滑减速。
- 在前进/倒车切换点附近 `transition_decel_distance` 范围内主动减速，降低冲击。
- 末端限制为低速蠕行 `velocity_creep`，并在终点速度收敛至 0。

新增参数：

- `velocity_creep`：最终接近最小速度，默认 0.05 m/s。
- `decel_distance`：终点减速区长度，默认 1.5 m。
- `transition_decel_distance`：换向减速区长度，默认 0.5 m。

### 3. 基于曲率的转角输出

- 路径点增加曲率信息 `curvature`。
- 轨迹点前轮转角按几何关系计算：

  `delta = atan(wheel_base * curvature)`

- 相比相邻点差分法，更不依赖采样间距，低速倒车时更稳定。

### 4. 室内低速场景参数收敛

- `path_resolution` 从 0.1 m 调整为 0.05 m，末端采样更细。
- `velocity_forward` 调整为 0.3 m/s。
- `velocity_reverse` 调整为 -0.2 m/s。

推荐实车调参顺序：

1. 先固定控制参数，仅调 `final_approach_distance` 和 `velocity_creep`。
2. 再调 `decel_distance` 与 `transition_decel_distance`，抑制换向冲击。
3. 最后按车体响应微调 `path_resolution` 与基础速度。

# 通过服务设置目标位姿并立即触发规划
ros2 service call /reverse_parking_planner/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

# 使用已有目标重新规划
ros2 service call /reverse_parking_planner/trigger_planning std_srvs/srv/Trigger



