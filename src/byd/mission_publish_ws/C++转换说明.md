# C++ 包转换说明

本文档说明了两个 ROS2 Python 包到 C++ 的转换。

## 转换的包

### 1. byd_initialize_pose_service
**原始包**: `service_initialize_pose` (Python)
**新包**: `byd_initialize_pose_service` (C++)

**功能说明**：
- 通过调用 `/api/localization/initialize` 服务来初始化车辆位置
- 支持两种初始化模式：
  - 手动初始化：使用配置文件中指定的位置和姿态
  - 自动初始化：使用 GNSS 位置（当 `auto_initialize: true` 时）

**包结构**：
```
byd_initialize_pose_service/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS2 包依赖定义
├── include/
│   └── initialize_pose_node.hpp # 节点头文件
├── src/
│   └── initialize_pose_node.cpp # 节点实现（包含 main 函数）
├── launch/
│   └── initialize_pose.launch.py
└── config/
    └── initial_pose.yaml       # 配置参数文件
```

**关键功能对应**：

| Python 功能 | C++ 实现 |
|---------|---------|
| `__init__()` | `InitializePoseNode()` 构造函数 |
| 参数声明 | `declare_parameter()` |
| 参数获取 | `get_parameter().as_double()`、`.as_string()` 等 |
| 服务客户端 | `create_client<InitializeLocalization>()` |
| 日志输出 | `RCLCPP_INFO()`、`RCLCPP_ERROR()` 等 |
| 异步回调 | `async_send_request()` + std::bind 回调 |
| 节点关闭 | `rclcpp::shutdown()` |

**构建和运行**：
```bash
# 构建包
colcon build --packages-select byd_initialize_pose_service

# 运行节点
ros2 run byd_initialize_pose_service initialize_pose_node

# 使用 launch 文件
ros2 launch byd_initialize_pose_service initialize_pose.launch.py
```

---

### 2. byd_set_goal_service
**原始包**: `service_set_goal` (Python)
**新包**: `byd_set_goal_service` (C++)

**功能说明**：
- 通过调用 `/api/routing/set_route_points` 服务设置多个连续的目标点
- 监听 `/api/routing/state` 话题来检测是否到达目标
- 支持在每个目标点处等待指定的时间
- 支持循环导航（可选）

**包结构**：
```
byd_set_goal_service/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS2 包依赖定义
├── include/
│   └── set_goal_node.hpp       # 节点头文件
├── src/
│   └── set_goal_node.cpp       # 节点实现（包含 main 函数）
├── launch/
│   └── set_goal.launch.py
└── config/
    └── goal_points.yaml        # 配置参数文件
```

**关键功能对应**：

| Python 功能 | C++ 实现 |
|---------|---------|
| `__init__()` | `SetGoalNode()` 构造函数 |
| 参数声明和加载 | `declare_parameter()` + `load_goals()` |
| 状态订阅回调 | `subscription` + 回调函数 |
| 计时器 | `create_wall_timer()` |
| 异步服务调用 | `async_send_request()` + std::bind 回调 |
| 异常处理 | try-catch 块 |
| 日志输出 | `RCLCPP_INFO()`、`RCLCPP_WARN()`、`RCLCPP_ERROR()` |

**状态管理流程**：
1. 启动时加载配置中的所有目标点
2. 清除之前的路由，然后发送第一个目标点
3. 监听路由状态变化
4. 当检测到 `ROUTE_STATE_ARRIVED` 时：
   - 记录到达消息
   - 启动等待计时器
   - 等待完成后移动到下一个目标
5. 如果启用了 `loop`，到达最后一个目标后循环回到第一个

**构建和运行**：
```bash
# 构建包
colcon build --packages-select byd_set_goal_service

# 运行节点
ros2 run byd_set_goal_service set_goal_node

# 使用 launch 文件
ros2 launch byd_set_goal_service set_goal.launch.py
```

---

## 转换说明

### 核心差异

1. **参数处理**
   - Python: `self.get_parameter('name').get_parameter_value().double_value`
   - C++: `this->get_parameter("name").as_double()`

2. **计时器**
   - Python: `self.create_timer(wait_time, callback)`
   - C++: `this->create_wall_timer(std::chrono::milliseconds(ms), callback)`

3. **异步服务调用**
   - Python: `future = self.client.call_async(request)`
   - C++: `future = client_->async_send_request(request, std::bind(...))` + lambda/std::bind

4. **日志记录**
   - Python: `self.get_logger().info()` / `error()` / `warn()`
   - C++: `RCLCPP_INFO()` / `RCLCPP_ERROR()` / `RCLCPP_WARN()`

5. **消息创建**
   - Python: `Header()`, `Pose()` 等直接创建
   - C++: 相同的消息类型，但需要包含相应头文件

### 功能一致性

✅ 所有原始 Python 代码的功能都已转换到 C++
✅ 参数配置文件保持不变（YAML 格式）
✅ Launch 文件结构保持一致
✅ 日志输出和错误处理方式相同
✅ 异步操作和状态管理逻辑完全对应

---

## 配置参数

### initial_pose.yaml

用于 `byd_initialize_pose_service`:

```yaml
initialize_pose_node:
  ros__parameters:
    frame_id: 'map'              # 坐标系参考
    x: 0.0                       # 位置 X
    y: 0.0                       # 位置 Y
    z: 0.0                       # 位置 Z
    orientation_x: 0.0           # 方向 X (四元数)
    orientation_y: 0.0           # 方向 Y (四元数)
    orientation_z: 0.0           # 方向 Z (四元数)
    orientation_w: 1.0           # 方向 W (四元数)
    auto_initialize: false        # 使用 GNSS 自动初始化
```

### goal_points.yaml

用于 `byd_set_goal_service`:

```yaml
set_goal_node:
  ros__parameters:
    frame_id: 'map'
    allow_goal_modification: true
    loop: true                   # 循环导航

    goal_names: ['A', 'B']       # 目标名称列表

    goals:
      A:
        x: 26.59
        y: -100.26
        z: 1.49
        orientation_x: 0.0
        orientation_y: 0.0
        orientation_z: -0.707
        orientation_w: 0.707
        wait_time: 5.0           # 到达后等待 5 秒

      B:
        x: -77.49
        y: -94.93
        z: 3.88
        orientation_x: 0.006
        orientation_y: -0.006
        orientation_z: -0.704
        orientation_w: -0.710
        wait_time: 5.0
```

---

## 依赖项

两个 C++ 包都需要以下依赖：
- `rclcpp` - ROS2 C++ 客户端库
- `geometry_msgs` - 几何消息
- `std_msgs` - 标准消息
- `autoware_adapi_v1_msgs` - Autoware AD API 消息

这些依赖在 `package.xml` 中已定义。

---

## 编译检查

构建前可以检查代码风格和编译选项：

```bash
# 启用所有警告
cmake_minimum_required(VERSION 3.8)
# CMakeLists.txt 中已包含: -Wall -Wextra -Wpedantic

# 检查编译
colcon build --packages-select byd_initialize_pose_service byd_set_goal_service
```

