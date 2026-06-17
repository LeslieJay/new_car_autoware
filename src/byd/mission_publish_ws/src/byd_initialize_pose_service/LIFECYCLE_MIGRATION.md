# byd_initialize_pose_service 生命周期节点改造说明

## 改动说明

`byd_initialize_pose_service` 已从普通节点改造为生命周期节点，并且在初始化位置完成后不会关闭，而是继续保持活跃状态。

## 主要改动

### 1. 继承关系变化
- **之前**: `class InitializePoseNode : public rclcpp::Node`
- **之后**: `class InitializePoseNode : public rclcpp_lifecycle::LifecycleNode`

### 2. 生命周期回调函数

节点现在实现了以下生命周期回调：

```cpp
// 配置阶段：初始化资源（如创建服务客户端）
on_configure(const rclcpp_lifecycle::State & state)

// 激活阶段：启动功能（发送初始位置）
on_activate(const rclcpp_lifecycle::State & state)

// 停用阶段：停止功能但保留资源
on_deactivate(const rclcpp_lifecycle::State & state)

// 清理阶段：释放所有资源
on_cleanup(const rclcpp_lifecycle::State & state)
```

### 3. 初始化流程

**原始流程：**
```
构造函数 → 声明参数 → 创建客户端 → 等待服务 → 发送位置 → 回调中调用 rclcpp::shutdown()
```

**新的生命周期流程：**
```
构造函数 → 声明参数
↓
on_configure() → 创建客户端 → 等待服务（最多10次重试）
↓
on_activate() → 发送初始位置
↓
回调处理 → 完成后不关闭，节点继续运行
↓
executor.spin() → 节点保持活跃状态
```

### 4. 关键变化

#### 移除自动关闭
```cpp
// ❌ 旧代码 - initialize_response_callback()
rclcpp::shutdown();  // 这行被移除了

// ✅ 新代码 - initialize_response_callback()
// Keep the node active - do not shutdown
```

#### 服务等待改进
在 `on_configure()` 中添加了重试机制：
```cpp
int retry_count = 0;
while (!client_->wait_for_service(std::chrono::seconds(1)) && retry_count < 10) {
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting... (attempt %d/10)", ++retry_count);
}

if (retry_count >= 10) {
    RCLCPP_ERROR(this->get_logger(), "Service not available after 10 attempts");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}
```

#### 构造函数简化
只在构造函数中声明参数，不执行业务逻辑：
```cpp
InitializePoseNode::InitializePoseNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("initialize_pose_node", options)
{
  // 只声明参数
  this->declare_parameter<std::string>("frame_id", "map");
  // ...
  RCLCPP_INFO(this->get_logger(), "Lifecycle node initialized");
}
```

### 5. main 函数改动

**旧的 main 函数：**
```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitializePoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

**新的 main 函数：**
```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitializePoseNode>();

  // 创建执行器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // 手动触发生命周期转移
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // 自旋执行器（节点保持活跃）
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
```

## 依赖更新

### package.xml
添加了以下依赖：
- `rclcpp_lifecycle` - ROS2 生命周期支持
- `lifecycle_msgs` - 生命周期消息定义

### CMakeLists.txt
- 添加了 `find_package(rclcpp_lifecycle)` 和 `find_package(lifecycle_msgs)`
- 在 `ament_target_dependencies` 中添加了这两个包

## 生命周期状态流转图

```
UNCONFIGURED
    ↓ (trigger_transition: TRANSITION_CONFIGURE)
CONFIGURING → on_configure() → SUCCESS
    ↓
INACTIVE
    ↓ (trigger_transition: TRANSITION_ACTIVATE)
ACTIVATING → on_activate() → SUCCESS
    ↓
ACTIVE (节点运行，可订阅/发布/调用服务)
    ↓ (trigger_transition: TRANSITION_DEACTIVATE)
DEACTIVATING → on_deactivate() → SUCCESS
    ↓
INACTIVE
    ↓ (trigger_transition: TRANSITION_CLEANUP)
CLEANINGUP → on_cleanup() → SUCCESS
    ↓
UNCONFIGURED
```

## 使用方法

### 运行节点
```bash
ros2 run byd_initialize_pose_service initialize_pose_node
```

### 查看节点状态
```bash
ros2 lifecycle get /initialize_pose_node
```

### 手动管理生命周期（可选）
```bash
# 从 UNCONFIGURED → INACTIVE
ros2 lifecycle set /initialize_pose_node configure

# 从 INACTIVE → ACTIVE
ros2 lifecycle set /initialize_pose_node activate

# 从 ACTIVE → INACTIVE
ros2 lifecycle set /initialize_pose_node deactivate

# 从 INACTIVE → UNCONFIGURED
ros2 lifecycle set /initialize_pose_node cleanup
```

## 优点

1. **资源管理更优雅** - 通过生命周期回调系统地管理资源
2. **可控性更强** - 支持手动或远程管理节点状态
3. **不自动关闭** - 完成位置初始化后，节点继续运行，可以响应其他请求或命令
4. **错误恢复能力更强** - 可以在失败时重新配置和激活，而不是直接关闭
5. **与 ROS2 生态更兼容** - 支持标准的生命周期管理工具和监控

## 编译

```bash
colcon build --packages-select byd_initialize_pose_service --symlink-install
```

编译结果：✅ 成功，无错误或警告
