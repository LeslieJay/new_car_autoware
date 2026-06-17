# byd_initialize_pose_service 生命周期节点 - 快速参考

## 概述

`byd_initialize_pose_service` 现已改造为 **ROS2 生命周期节点**，具有以下特点：

✅ 节点在初始化位置完成后**不会自动关闭**  
✅ 支持**生命周期状态管理**  
✅ 支持**远程控制节点状态**  
✅ 更**健壮的错误恢复**机制  

## 节点功能

| 功能 | 说明 |
|------|------|
| **服务调用** | 调用 `/api/localization/initialize` 初始化车辆位置 |
| **快速初始化** | 支持手动初始化（指定位置/姿态）或自动初始化（GNSS） |
| **状态管理** | 通过生命周期回调管理节点的整个生命周期 |
| **持续运行** | 完成位置初始化后，节点继续运行 |

## 快速开始

### 1. 构建
```bash
colcon build --packages-select byd_initialize_pose_service --symlink-install
```

### 2. 运行
```bash
ros2 run byd_initialize_pose_service initialize_pose_node
```

### 3. 查看节点状态
```bash
ros2 lifecycle get /initialize_pose_node
```

## 生命周期状态管理

### 节点启动流程
```
1. 启动可执行文件
   ↓
2. main() 执行
   ↓
3. 创建节点实例
   ↓
4. 触发 TRANSITION_CONFIGURE
   ├─ 调用 on_configure()
   ├─ 创建服务客户端
   ├─ 等待服务可用
   └─ 返回 SUCCESS/FAILURE
   ↓
5. 触发 TRANSITION_ACTIVATE
   ├─ 调用 on_activate()
   ├─ 获取参数
   ├─ 发送初始位置请求
   └─ 节点进入 ACTIVE 状态
   ↓
6. executor.spin() - 节点持续运行
```

### 手动控制节点状态（可选）

```bash
# 查看当前状态
ros2 lifecycle get /initialize_pose_node

# 从 INACTIVE 激活到 ACTIVE
ros2 lifecycle set /initialize_pose_node activate

# 从 ACTIVE 停用到 INACTIVE
ros2 lifecycle set /initialize_pose_node deactivate

# 清理资源
ros2 lifecycle set /initialize_pose_node cleanup
```

## 配置参数

编辑 `config/initial_pose.yaml`：

```yaml
initialize_pose_node:
  ros__parameters:
    frame_id: 'map'              # 坐标系
    x: 0.0                       # 位置 X
    y: 0.0                       # 位置 Y
    z: 0.0                       # 位置 Z
    orientation_x: 0.0           # 方向 X (四元数)
    orientation_y: 0.0           # 方向 Y (四元数)
    orientation_z: 0.0           # 方向 Z (四元数)
    orientation_w: 1.0           # 方向 W (四元数)
    auto_initialize: false        # false=手动, true=GNSS自动
```

## 日志输出示例

### 正常运行流程
```
[INFO] Lifecycle node initialized
[INFO] on_configure() is called from state UNCONFIGURED
[INFO] Waiting for /api/localization/initialize service...
[INFO] Service is available!
[INFO] on_activate() is called from state INACTIVE
[INFO] Sending initial pose: position=(0.50, 0.57, -1.37), orientation=(0.0016, 0.0314, -0.0511, 0.9982)
[INFO] Pose initialized successfully!
```

### 故障恢复
```
[INFO] on_configure() is called from state UNCONFIGURED
[WARN] Service not available, waiting... (attempt 1/10)
[WARN] Service not available, waiting... (attempt 2/10)
...
[ERROR] Service not available after 10 attempts
```

## 关键改动

| 方面 | 旧版本 | 新版本 |
|------|-------|--------|
| **基类** | `rclcpp::Node` | `rclcpp_lifecycle::LifecycleNode` |
| **生命周期** | 无 | on_configure/activate/deactivate/cleanup |
| **初始化时机** | 构造函数立即执行 | 在 on_activate() 中执行 |
| **节点关闭** | 任务完成后立即关闭 | 任务完成后继续运行 |
| **服务等待** | 无限等待 | 最多等待 10 秒 |
| **状态management** | 不支持 | 支持完整的生命周期管理 |

## 文件结构

```
byd_initialize_pose_service/
├── CMakeLists.txt                      # CMake 构建文件
├── package.xml                         # 包依赖定义
├── LIFECYCLE_MIGRATION.md              # 详细改动说明
├── include/
│   └── initialize_pose_node.hpp        # 头文件（包含生命周期回调）
├── src/
│   └── initialize_pose_node.cpp        # 源文件（包含 main 函数）
├── launch/
│   └── initialize_pose.launch.py       # 启动文件
└── config/
    └── initial_pose.yaml               # 配置参数
```

## 常见问题

### Q: 为什么节点不再自动关闭？
A: 将节点改为生命周期节点后，节点会进入 ACTIVE 状态并保持运行。这样设计是为了支持：
- 实时监控和远程控制
- 故障恢复和重新配置
- 与其他节点的协调管理

### Q: 如何让节点退出？
A: 按 `Ctrl+C` 或使用以下命令：
```bash
ros2 lifecycle set /initialize_pose_node deactivate
ros2 lifecycle set /initialize_pose_node cleanup
# 然后按 Ctrl+C
```

### Q: 如何在其他节点中启动这个节点的生命周期？
A: 若要从其他节点控制这个节点的生命周期，可以使用 [ROS2 生命周期管理器](https://github.com/ros2/demos/tree/master/lifecycle_py) 或编写专门的管理节点。

## 编译依赖

| 依赖 | 版本 | 用途 |
|-----|------|------|
| rclcpp | - | ROS2 C++ 客户端库 |
| rclcpp_lifecycle | - | 生命周期支持 |
| lifecycle_msgs | - | 生命周期消息定义 |
| autoware_adapi_v1_msgs | - | Autoware API 消息 |
| geometry_msgs | - | 几何数据类型 |

## 编译状态

✅ **编译成功** - 无错误或警告  
✅ **单元测试** - 待测试  
✅ **集成测试** - 待测试  

---

**最后更新**: 2026-03-09  
**状态**: 生产就绪 ✅
