• MRM（Minimal Risk Maneuver，最小风险操作）的作用是：当 Autoware 判断当前自动驾驶模式已经不再安全可用时，不再继续正常自动驾驶，而是选择一个仍可用的降级动作，让车辆进入风险尽可能低的状态。

  当前项目代码支持三种 MRM：

  1. PULL_OVER：靠边停车
  2. COMFORTABLE_STOP：舒适减速停车
  3. EMERGENCY_STOP：紧急制动停车

  不过当前项目参数实际配置为：

  use_pull_over: false
  use_comfortable_stop: true

  因此当前有效策略是：

  正常自动驾驶
      ↓ 当前模式不可用
  优先舒适停车
      ↓ 舒适停车不可用、调用失败或状态超时
  紧急停车

  配置见 src/launcher/autoware_launch/autoware_launch/config/system/mrm_handler/mrm_handler.param.yaml:1。

  ## 一、MRM 接收哪些数据

  ### 1. 系统模式可用性

  /system/operation_mode/availability

  类型：

  tier4_system_msgs/msg/OperationModeAvailability

  这是最核心的触发输入，包含：

  - autonomous
  - stop
  - local
  - remote
  - comfortable_stop
  - emergency_stop
  - pull_over

  这些字段由 diagnostics graph 根据各模块诊断状态计算。

  例如：

  autonomous=false
  comfortable_stop=true
  emergency_stop=true

  表示正常自动驾驶模式已经不可用，但舒适停车和紧急停车仍可执行。

  ### 2. 当前 Operation Mode

  /api/operation_mode/state

  用于确定车辆当前处于：

  - STOP
  - AUTONOMOUS
  - LOCAL
  - REMOTE
  - UNKNOWN

  MRM 会用当前模式去查询 OperationModeAvailability 中对应字段。逻辑见 src/universe/autoware_universe/system/autoware_mrm_handler/src/mrm_handler/mrm_handler_core.cpp:601。

  ### 3. 底盘控制模式

  /vehicle/status/control_mode

  用于判断底盘是否处于 AUTONOMOUS。只有处于自动控制状态，MRM 状态机才会从 NORMAL 进入 MRM_OPERATING。

  ### 4. 定位和车速

  /localization/kinematic_state

  MRM 主要使用其中的纵向速度判断车辆是否停稳。

  当前代码的停车阈值非常小：

  abs(speed) < 0.001 m/s

  见 src/universe/autoware_universe/system/autoware_mrm_handler/src/mrm_handler/mrm_handler_core.cpp:550。

  ### 5. 各 MRM Operator 状态

  /system/mrm/comfortable_stop/status
  /system/mrm/emergency_stop/status
  /system/mrm/pull_over_manager/status

  用于确认相应降级行为节点是否可用、正在运行、成功或失败。

  ### 6. 当前档位命令

  /control/command/gear_cmd

  MRM 正常情况下保持原档位；如果配置了停车后进 P 档，车辆停稳后会输出 PARK。

  当前项目：

  use_parking_after_stopped: false

  所以触发 MRM 后不会自动切换 P 档。

  ## 二、MRM 怎么触发

  ### 自动触发条件

  MRM 的紧急状态判断是：

  当前 Operation Mode 不可用
  或 emergency holding 生效
  或 OperationModeAvailability 消息超时

  对应代码：

  return !isAvailableCurrentOperationMode() ||
         is_emergency_holding_ ||
         is_operation_mode_availability_timeout;

  见 src/universe/autoware_universe/system/autoware_mrm_handler/src/mrm_handler/mrm_handler_core.cpp:558。

  最常见触发过程：

  某个模块报 ERROR
    ↓
  diagnostic graph 判断 autonomous 模式不可用
    ↓
  /system/operation_mode/availability 中 autonomous=false
    ↓
  MRM Handler 检测当前模式为 AUTONOMOUS
    ↓
  状态 NORMAL → MRM_OPERATING
    ↓
  选择 Comfortable Stop 或 Emergency Stop

  ### Availability 超时触发

  当前参数：

  timeout_operation_mode_availability: 0.5

  如果 /system/operation_mode/availability 超过 0.5 秒没有更新，MRM 直接选择 EMERGENCY_STOP，不会等待舒适停车。

  ### 行为选择优先级

  当前代码的选择顺序为：

  1. Availability 消息超时：EMERGENCY_STOP
  2. pull_over=true 且配置允许：PULL_OVER
  3. comfortable_stop=true 且配置允许：COMFORTABLE_STOP
  4. 其他情况：EMERGENCY_STOP

  当前项目关闭了 pull-over，所以通常是：

  COMFORTABLE_STOP → EMERGENCY_STOP

  ### Operator 调用失败

  如果舒适停车服务调用失败、超时或取消失败，handler 会调用：

  /system/mrm/emergency_stop/operate

  转为紧急停车。当前服务调用超时只有：

  timeout_call_mrm_behavior: 0.01
  timeout_cancel_mrm_behavior: 0.01

  即 10 ms，这个值比较严格，系统高负载时存在误判超时并升级到 emergency stop 的风险。

  ## 三、触发后有哪些影响

  ### 1. Comfortable Stop

  handler 调用：

  /system/mrm/comfortable_stop/operate

  Comfortable Stop Operator 向规划链发布速度限制：

  /planning/scenario_planning/max_velocity_candidates

  内容等价于：

  目标最大速度 = 0
  最小加速度 = -1.0 m/s²
  jerk 范围 = [-0.3, 0.3] m/s³

  因此它不是直接覆盖底盘控制命令，而是要求规划速度平滑降到 0。

  配置见 src/launcher/autoware_launch/autoware_launch/config/system/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator.param.yaml:1。

  影响：

  - 正常轨迹仍可能继续发布。
  - 轨迹速度会被限制并逐渐降到 0。
  - 减速度较小，停车相对平稳。
  - 需要规划和控制链仍然正常工作。
  - 如果规划链本身故障，comfortable stop 可能无法可靠停车。

  ### 2. Emergency Stop

  handler 调用：

  /system/mrm/emergency_stop/operate

  Emergency Stop Operator 接收正常控制命令：

  /control/command/control_cmd

  然后输出紧急控制命令：

  /system/emergency/control_cmd

  当前参数：

  target_acceleration: -2.5
  target_jerk: -1.5

  即目标减速度为 -2.5 m/s²，以 -1.5 m/s³ 的 jerk 逐渐进入制动。

  配置见 src/launcher/autoware_launch/autoware_launch/config/system/mrm_emergency_stop_operator/mrm_emergency_stop_operator.param.yaml:1。

  影响：

  - 紧急控制命令进入 command gate。
  - 正常规划轨迹不再决定纵向停车行为。
  - 目标速度降为 0。
  - 制动强度明显大于 comfortable stop。
  - 横向命令初始基于最后一条正常控制命令。
  - 如果正常控制命令没有到达，紧急停车节点可能缺少合理的初始控制状态。

  ### 3. 打开双闪

  当前配置：

  turning_hazard_on:
    emergency: true

  触发紧急状态后发布：

  /system/emergency/hazard_lights_cmd

  命令为 ENABLE。

  同时转向灯命令被设为 DISABLE/NO_COMMAND，避免普通转向灯与双闪冲突。

  ### 4. 档位处理

  输出：

  /system/emergency/gear_cmd

  当前配置不会在停车后切 P 档，因此一般保持 MRM 触发前的档位。

  ### 5. 发布 MRM 状态

  /system/fail_safe/mrm_state

  状态机包括：

  NORMAL
  MRM_OPERATING
  MRM_SUCCEEDED
  MRM_FAILED

  行为包括：

  NONE
  PULL_OVER
  COMFORTABLE_STOP
  EMERGENCY_STOP

  对于 comfortable/emergency stop，只要检测到车辆速度低于 0.001 m/s，状态就会转为 MRM_SUCCEEDED。
