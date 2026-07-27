# Planning 障碍物相关模块分析

本文档分析三个与 BYD 低速 AGV 障碍物处理相关的模块/配置：

- [`autoware_surround_obstacle_checker`](../autoware_surround_obstacle_checker/)
- [`autoware_behavior_path_simple_avoidance_module`](./autoware_behavior_path_simple_avoidance_module/)
- [`autoware_behavior_path_simple_lane_change_avoidance_module`](./autoware_behavior_path_simple_lane_change_avoidance_module/)

第三个用户给出的路径是 launch 参数副本：

[`autoware_behavior_path_simple_avoidance_module/simple_avoidance.param.yaml`](../../../../launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_simple_avoidance_module/simple_avoidance.param.yaml)

它不是源码包本体，但它是 `autoware_launch` 启动时实际传给 behavior path planner 的参数文件之一。

## 1. 总体定位

这三块不是同一层的功能。

| 模块 | 所在层级 | 核心动作 | 主要输出 | 失败/不可行时 |
|------|----------|----------|----------|---------------|
| `surround_obstacle_checker` | planning safety / velocity limit | 检查车身周围是否有危险物，必要时请求停车 | velocity limit、clear command、command gate stop、status、ready | 进入 STOP 或保持 PASS，取决于状态机与参数 |
| `simple_avoidance` | behavior path planner slot2 | 在当前车道内用 PathShifter 做横向偏移绕障 | 偏移后的 `PathWithLaneId` 和扩展 drivable area | 透传上游路径 |
| `simple_lane_change_avoidance` | behavior path planner slot2 | 借 route 上的相邻车道绕障，再回原参考路径 | 偏移后的 `PathWithLaneId` 和包含邻道的 drivable area | 透传上游路径 |

一句话理解：

- `surround_obstacle_checker` 是“停住/别起步”的保护模块。
- `simple_avoidance` 是“本车道内绕过去”的路径模块。
- `simple_lane_change_avoidance` 是“本车道不够就借 route 邻道”的路径模块。

## 2. 启动和参数来源

### 2.1 behavior path planner 模块加载

`behavior_planning.launch.xml` 会根据 launch 参数拼出 `behavior_path_planner_launch_modules`。其中：

- `launch_simple_avoidance=true` 时加载 `SimpleAvoidanceModuleManager`
- `launch_simple_lc_avoidance=true` 时加载 `SimpleLaneChangeAvoidanceModuleManager`

来源：

- [`behavior_planning.launch.xml`](../../../../launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml)
- [`tier4_planning_component.launch.xml`](../../../../launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml)

当前 `default_preset.yaml` 中：

```yaml
launch_static_obstacle_avoidance: "false"
launch_simple_avoidance: "true"
launch_simple_lc_avoidance: "false"
```

也就是说默认只启用 `simple_avoidance`，不启用借道版。虽然 `behavior_planning.launch.xml` 里 `launch_simple_lc_avoidance` 的 XML 默认值是 `true`，但 `autoware_launch` 会先 include `default_preset.yaml`，实际默认应以 preset 为准。

### 2.2 slot2 竞争关系

`scene_module_manager.param.yaml` 把这些模块都放在 slot2：

- `avoidance_by_lane_change`
- `simple_lane_change_avoidance`
- `static_obstacle_avoidance`
- `simple_avoidance`
- `lane_change_left/right`
- 其他 lane change / side shift 相关模块

来源：

[`scene_module_manager.param.yaml`](../../../../launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/scene_module_manager.param.yaml)

slot2 内数字优先级越靠前越高。配置中 `simple_lane_change_avoidance` 排在 `simple_avoidance` 前面，但模块是否参与还取决于 preset 是否启用。为了避免同一周期多个绕障策略竞争，实车调试建议同一类绕障只启用一种：

```yaml
launch_simple_avoidance: "true"
launch_simple_lc_avoidance: "false"
```

或：

```yaml
launch_simple_avoidance: "false"
launch_simple_lc_avoidance: "true"
```

## 3. `surround_obstacle_checker`

### 3.1 模块职责

`surround_obstacle_checker` 是一个 ROS component 节点，节点名为 `surround_obstacle_checker_node`。它每 100 ms 轮询一次输入数据，判断车身周围是否存在危险物或输入是否失效，并发布停车相关命令。

主要源码：

- [`src/node.hpp`](../autoware_surround_obstacle_checker/src/node.hpp)
- [`src/node.cpp`](../autoware_surround_obstacle_checker/src/node.cpp)
- [`param/surround_obstacle_checker_node_parameters.yaml`](../autoware_surround_obstacle_checker/param/surround_obstacle_checker_node_parameters.yaml)

### 3.2 输入输出

输入：

- `~/input/odometry`
- `~/input/pointcloud`
- `~/input/objects`
- TF

输出：

- `~/output/max_velocity`
- `~/output/velocity_limit_clear_command`
- `~/output/status`
- `~/output/ready`
- `~/debug/processing_time_ms`
- debug marker / footprint

如果 `request_command_gate_stop=true`，还会调用服务：

```text
/control/vehicle_cmd_gate/set_stop
```

### 3.3 状态机

内部只有两个状态：

| 状态 | 含义 |
|------|------|
| `PASS` | 当前不要求停车 |
| `STOP` | 当前要求停车或保持停车 |

主循环逻辑：

1. 轮询 odometry、pointcloud、objects，并记录本地接收时间。
2. 如果 `fail_safe_on_data_timeout=true`，检查必需输入是否缺失或超过 `data_timeout_sec`。
3. 若输入可用，计算最近障碍物。
4. 根据 `is_obstacle_found`、车辆是否停止、当前状态、`state_clear_time` 得出是否需要 STOP。
5. `PASS -> STOP` 时发布 `VelocityLimit{max_velocity=0}`。
6. `STOP -> PASS` 时发布 `VelocityLimitClearCommand`。
7. 根据 STOP/PASS 调用 command gate `set_stop`。
8. 发布 `status` 和 debug 信息。

### 3.4 障碍物距离的真实含义

配置里的 `surround_check_front_distance`、`surround_check_side_distance`、`surround_check_back_distance` 不是直接拿来和障碍物距离比较。

源码做法是先把自车 footprint 按 front/side/back 扩大：

- front：`vehicle_info_.max_longitudinal_offset_m + front_margin`
- rear：`vehicle_info_.rear_overhang_m + back_margin`
- width：`vehicle_info_.vehicle_width_m + side_margin * 2`

然后计算扩大后的 ego polygon 到点云点或动态物体 polygon 的 Boost Geometry 距离。`PASS` 状态下阈值接近 0，意思是“障碍物进入扩大后的 footprint 就触发”；`STOP` 状态下阈值使用 `surround_check_hysteresis_distance`，用于迟滞释放。

这个细节很重要：把 front distance 设置成 0，不是禁用前方检测，而是前方检测区域不再额外外扩，只剩车辆本体前缘。

### 3.5 数据超时和 fail-safe

`isInputUnsafe()` 会检查：

- odometry 是否缺失/超时
- 如果启用了动态物体检查，objects 是否缺失/超时
- 如果启用了点云检查，pointcloud 是否缺失/超时

当 `fail_safe_on_data_timeout=true` 且输入不安全时，`is_obstacle_found` 会被视为 true，状态原因会是：

- `odometry_timeout`
- `objects_timeout`
- `pointcloud_timeout`

但还要注意 `stop_only_when_stopped`。如果它为 true，并且当前车辆不是 stopped，`isStopRequired()` 会直接返回 false。也就是说：

```yaml
fail_safe_on_data_timeout: true
stop_only_when_stopped: true
```

更像是“输入异常时阻止已停车辆起步/保持停止”，不是“车辆运动中立刻触发停车”。

### 3.6 ready 话题语义

`~/output/ready` 是 transient local 的 `std_msgs/Bool`。

- `request_command_gate_stop=false` 时，构造函数里会直接发布 ready。
- `request_command_gate_stop=true` 时，只有 command gate `set_stop` 请求成功返回后才发布 ready。

BYD launch 中把它 remap 到：

```text
/byd/pedestrian_safety_stop/ready
```

这表示“节点已启动并且 command gate stop 请求链路已至少成功应答过”，不是“当前没有障碍物”。

### 3.7 BYD pedestrian safety stop 配置

BYD 配置文件：

[`src/byd/launch/config/pedestrian_safety_stop.param.yaml`](../../../../byd/launch/config/pedestrian_safety_stop.param.yaml)

当前关键值：

| 参数 | 当前值 | 影响 |
|------|--------|------|
| `unknown.enable_check` | `true` | 检查未知物体 |
| `pedestrian.enable_check` | `true` | 检查行人 |
| 其他对象类型 | `false` | 不检查 car/truck/bicycle 等 |
| `pointcloud.enable_check` | `false` | 不检查点云 |
| `surround_check_front_distance` | `0` | 前方不额外扩展，只按车辆前缘 |
| `surround_check_side_distance` | `1.0` | 左右各扩 1 m |
| `surround_check_back_distance` | `2.0` | 后方扩 2 m |
| `surround_check_hysteresis_distance` | `0.5` | STOP 释放迟滞 |
| `state_clear_time` | `2.0` | 障碍消失后保持 STOP 的时间 |
| `stop_only_when_stopped` | `true` | 只阻止已停车辆起步/保持停止 |
| `fail_safe_on_data_timeout` | `true` | 必需输入缺失/超时时进入危险判定 |
| `request_command_gate_stop` | `true` | 同时请求 vehicle command gate stop |

## 4. `simple_avoidance`

### 4.1 模块职责

`simple_avoidance` 是 behavior path planner 的 `SceneModuleInterface` 插件。它不发布停车点，不修改速度，只修改几何路径和 drivable area。

主要源码：

- [`src/manager.cpp`](./autoware_behavior_path_simple_avoidance_module/src/manager.cpp)
- [`src/scene.cpp`](./autoware_behavior_path_simple_avoidance_module/src/scene.cpp)
- [`src/utils.cpp`](./autoware_behavior_path_simple_avoidance_module/src/utils.cpp)
- [`include/.../data_structs.hpp`](./autoware_behavior_path_simple_avoidance_module/include/autoware/behavior_path_simple_avoidance_module/data_structs.hpp)

### 4.2 参数来源

源码包默认参数：

[`config/simple_avoidance.param.yaml`](./autoware_behavior_path_simple_avoidance_module/config/simple_avoidance.param.yaml)

Autoware launch 实际传入的参数副本：

[`autoware_launch/.../autoware_behavior_path_simple_avoidance_module/simple_avoidance.param.yaml`](../../../../launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_simple_avoidance_module/simple_avoidance.param.yaml)

需要特别注意：launch 副本里 `min_forward_distance` 当前是 `3.0`，源码包默认文件里是 `0.5`。实车/launch 场景通常应看 launcher 目录下的参数副本。

### 4.3 每周期数据准备

`updateData()` 做四件事：

1. 取上游模块输出路径。
2. 用 `extendBackwardLength()` 拼接历史后向路径，保证 PathShifter 的 shift line 在自车附近不会缺点。
3. 按 1 m 间隔重采样参考路径。
4. 从 route handler 获取当前 route lanelet sequence，供目标是否在车道内判断和 drivable area 生成。

### 4.4 目标筛选

`detectTarget()` 只选择一个最近目标。筛选顺序：

1. 必须有 dynamic objects，且 reference path 非空。
2. 目标速度 `< th_moving_speed`。
3. 若 current lanelets 非空，目标 polygon 必须与当前 route lanelet overlap。
4. 纵向距离在 `[min_forward_distance, max_forward_distance]` 内。
5. 横向 overlap 风险成立：

```text
abs(lateral_offset) - object_half_width
  < ego_half_width + lateral_margin + hysteresis
```

普通检测时 hysteresis 为 0；刷新已锁定目标时使用 `target_hold_lateral_hysteresis`。

多个候选目标同时满足时，只取纵向最近的一个。

### 4.5 目标保持

与借道版不同，`simple_avoidance` 有 UUID 级别的目标保持逻辑：

- 首次检测到目标后写入 `active_target_`。
- 后续优先按 UUID 刷新同一个目标。
- 如果短时检测不到，但没有超过 `target_lost_time_threshold`，继续持有旧目标。
- 如果目标已通过，`getActiveTargetOrHeldTarget()` 返回空，但模块会继续处理残留 shift line，直到回正完成。

这解决了 shifted path 导致目标横向关系瞬间变化、模块过早退出的问题。

### 4.6 侧移方向和侧移量

`calcShiftLength()` 使用障碍物靠近参考路径的一侧，而不是远侧：

```text
object_near_edge =
  lateral_offset >= 0 ? lateral_offset - object_half_width
                      : lateral_offset + object_half_width

required_clearance =
  max(0, abs(object_near_edge)) + ego_half_width + lateral_margin
```

方向遵循 PathShifter 约定：

- 障碍物在左侧，`lateral_offset >= 0`，生成负 shift，向右绕。
- 障碍物在右侧，`lateral_offset < 0`，生成正 shift，向左绕。

如果 `abs(shift_length) > max_shift_length`，会截断到 `max_shift_length`，但只要仍有 `remaining_gap > 0` 就返回 `NO_ROOM`，最终透传上游路径。

### 4.7 可行性检查

`checkFeasibility()` 只检查纵向空间是否够完成侧移：

```text
jerk_distance = calc_longitudinal_dist_from_jerk(
  abs(shift_length),
  shifting_lateral_jerk,
  max(abs(ego_speed), min_shifting_speed)
)

dist_to_shift_end = min_prepare_distance + max(jerk_distance, min_shifting_distance)
dist_to_obstacle  = target.longitudinal_distance - object_half_length - lateral_margin
```

如果 `dist_to_shift_end > dist_to_obstacle`，返回 `INSUFFICIENT_DISTANCE`。

这里 `lateral_margin` 也被复用为障碍物前沿的纵向 buffer。调大它会同时让横向更保守、纵向更难通过可行性检查。

### 4.8 PathShifter 输出

可行时构造两条 shift line：

| Shift line | 起点 | 终点 | 偏移变化 |
|------------|------|------|----------|
| avoid | ego + `min_prepare_distance` | avoid start + max(jerk distance, min shifting distance) | 当前偏移到目标偏移 |
| return | 障碍物中心 + 半长 + `return_distance_after_object` | return start + max(jerk distance, min shifting distance) | 目标偏移回 0 |

然后调用 `path_shifter_.generate()`，补齐 path orientation，并扩展 drivable area。

### 4.9 结束条件

`canTransitSuccessState()` 要求：

- active target 不存在，或者已通过；
- shift lines 已清空；
- ego 不在任何 shift line 上；
- PathShifter base offset 接近 0；
- 当前 ego shift 接近 0。

阈值由 `lateral_execution_threshold` 控制。

## 5. `simple_lane_change_avoidance`

### 5.1 模块职责

`simple_lane_change_avoidance` 也是 behavior path planner 的 slot2 插件。它不是完整 Autoware lane change 模块，也没有邻道来车安全检查。它的核心是假设 route 上可借邻道安全可用，通过 PathShifter 偏移到邻道中心附近，绕过目标后回到原参考路径。

主要源码：

- [`src/manager.cpp`](./autoware_behavior_path_simple_lane_change_avoidance_module/src/manager.cpp)
- [`src/scene.cpp`](./autoware_behavior_path_simple_lane_change_avoidance_module/src/scene.cpp)
- [`src/utils.cpp`](./autoware_behavior_path_simple_lane_change_avoidance_module/src/utils.cpp)
- [`include/.../data_structs.hpp`](./autoware_behavior_path_simple_lane_change_avoidance_module/include/autoware/behavior_path_simple_lane_change_avoidance_module/data_structs.hpp)

### 5.2 目标筛选

目标筛选基本等同 `simple_avoidance`：

- 低速目标；
- 当前 route lanelet 内；
- 前方指定纵向距离内；
- 与自车横向 envelope 存在 overlap；
- 多目标取最近一个。

区别是它没有 UUID 短时保持、目标迟滞和详细 no-target 诊断结构。

### 5.3 借道方向

`getAvoidanceDirection()` 的规则：

| 障碍物位置 | `lateral_offset` | 借道方向 |
|------------|------------------|----------|
| 左侧 | `>= 0` | RIGHT |
| 右侧 | `< 0` | LEFT |

也就是障碍在左，向右借道；障碍在右，向左借道。

### 5.4 邻道查找

`findAdjacentLane()`：

1. 从 route handler 获取 ego 当前 lanelet。
2. 根据方向调用 `getLeftLanelet()` 或 `getRightLanelet()`。
3. 要求邻道存在。
4. 要求邻道本身在 route 上。
5. 获取邻道 lanelet sequence。

如果邻道不存在或不在 route 上，返回 `NO_ADJACENT_LANE`，模块透传。

### 5.5 借道偏移量

`calcLaneShift()` 使用 lanelet arc coordinate 的 lateral distance：

```text
raw = current_lane_distance - adjacent_lane_distance
shift_length = raw + sign(raw) * lateral_margin
```

单元测试体现了 PathShifter 符号：

- 左邻道距离为 `-3.5` 时，shift 为 `+3.8`。
- 右邻道距离为 `+3.5` 时，shift 为 `-3.8`。

如果 `abs(shift_length) < 0.1`，认为没有有效邻道偏移，返回 `NO_ADJACENT_LANE`。

### 5.6 可行性和输出

可行性检查与 `simple_avoidance` 同形式，只是 shift length 来自邻道中心距离，而不是目标包络 clearance。

可行时也生成 avoid + return 两条 shift line。输出路径后，`adjustDrivableArea()` 使用 `generateDrivableLanesWithAdjacent()` 把当前 lanelets 和邻道 lanelets 组合起来，再做 lanelet expansion。

### 5.7 状态保持和退出

借道版的状态保持比车道内版简单：

- 如果 `path_shifter_` 里已有 shift lines，本周期继续 `generate()` 并输出。
- 新 maneuver 只在 shift lines 为空时初始化。
- `canTransitSuccessState()` 要求没有新 target、shift lines 为空、当前 shift 接近 0。

它没有 `target_lost_time_threshold` 和 UUID hold。因此感知抖动时，行为稳定性依赖已经生成的 shift lines 和 PathShifter 内部状态。

## 6. 三者协作关系

### 6.1 推荐组合

当前默认组合是：

```yaml
launch_simple_avoidance: "true"
launch_simple_lc_avoidance: "false"
```

适用于本车道宽度足够、希望优先做车道内偏移绕障的场景。

如果现场地图 route 明确包含可借邻道，并且本车道内 `simple_avoidance` 经常 `NO_ROOM`，可以切换为：

```yaml
launch_simple_avoidance: "false"
launch_simple_lc_avoidance: "true"
```

`surround_obstacle_checker` 可以与任一路径绕障模块并行使用，因为它不是 slot2 路径模块。它的职责是提供额外停止保护，尤其适合近车身 pedestrian/unknown 防护。

### 6.2 不建议的组合

不建议同时启用：

```yaml
launch_simple_avoidance: "true"
launch_simple_lc_avoidance: "true"
```

原因：

- 二者都在 slot2；
- 二者都会看同一批动态 objects；
- 二者不可行时都会透传，不插停车点；
- slot 内优先级和同时执行策略会让调试现象变复杂。

如果确实需要“本车道可绕就本车道，不能绕再借道”的级联策略，应单独设计一个仲裁策略，而不是简单同时打开两个模块。

## 7. 调试速查

### 7.1 surround obstacle checker

```bash
ros2 topic echo /byd/pedestrian_safety_stop/status --once
ros2 topic echo /byd/pedestrian_safety_stop/ready --once
ros2 service list | grep /control/vehicle_cmd_gate/set_stop
```

重点看 status values：

- `state`
- `command_gate_acknowledged`
- `odometry_age_sec`
- `objects_age_sec`
- `pointcloud_age_sec`
- `nearest_distance`

### 7.2 simple avoidance

```bash
grep 'SIMPLE_AVOIDANCE' your.log
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/simple_avoidance --once
```

常见 pass-through：

| reason | 典型含义 |
|--------|----------|
| `no_target` | 没有目标通过速度/车道/纵向/横向 overlap 过滤 |
| `infeasible_no_room` | 所需侧移超过 `max_shift_length` |
| `infeasible_distance` | 障碍物太近，准备 + 侧移距离不够 |
| `path_generation_failed` | PathShifter 生成失败，常见于路径点不足或 shift line 区间异常 |

### 7.3 simple lane change avoidance

```bash
grep 'SIMPLE_LC_AVOIDANCE' your.log
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/simple_lane_change_avoidance --once
```

常见 pass-through：

| reason | 典型含义 |
|--------|----------|
| `no_target` | 没有目标通过筛选 |
| `no_adjacent_lane` | 目标方向对应邻道不存在，或邻道不在 route 上 |
| `infeasible_distance` | 借道侧移距离太长，纵向空间不足 |
| `path_generation_failed` | PathShifter 生成失败 |

## 8. 关键风险和边界

1. `simple_avoidance` 和 `simple_lane_change_avoidance` 都不做纵向减速/停车规划。车仍停住时，应同时检查 behavior velocity planner、obstacle stop、vehicle command gate、surround obstacle checker。
2. `simple_lane_change_avoidance` 不检查邻道动态交通，不适合开放道路。
3. `surround_obstacle_checker` 的 `ready` 不是“安全无障碍”，只是“节点/command gate 请求链路 ready”。
4. `surround_obstacle_checker` 的 `fail_safe_on_data_timeout=true` 若搭配 `stop_only_when_stopped=true`，主要是防止已停车辆起步，不是运动中强制停车。
5. launch 参数副本和源码包默认参数可能不一致。实车启动时优先检查 `src/launcher/autoware_launch/autoware_launch/config/...` 下的参数副本。

## 9. 选型建议

| 场景 | 建议 |
|------|------|
| 近车身行人/未知物防护 | 开启 `surround_obstacle_checker` 的 BYD pedestrian safety 配置 |
| 本车道足够宽，障碍物静止或近似静止 | 使用 `simple_avoidance` |
| 本车道绕不开，但 route 上存在可借邻道 | 使用 `simple_lane_change_avoidance`，并关闭 `simple_avoidance` |
| 开放道路、需要邻道来车/对向车安全判断 | 不建议用这两个 simple 模块，应回到完整 avoidance / lane change 策略 |
| 需要不可行时主动停车 | 不应依赖 simple path 模块，需要 behavior velocity planner 或 command gate/安全模块兜底 |
