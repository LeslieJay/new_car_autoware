# Autoware 原生换车道绕障机制研究

## 研究范围与结论摘要

本文只依据仓库内的一手源码和模块文档，研究以下实现：

- `autoware_behavior_path_avoidance_by_lane_change_module`：换车道绕障的触发、目标筛选和方向决策；
- `autoware_behavior_path_lane_change_module`：实际换道路径、安全检查、停止、输出和状态处理；
- `behavior_path_planner_common` 与 `PlannerManager`：模块生命周期、候选/批准队列、RTC 和路线切换。

原生“换车道绕障”并不是独立重写一套换道算法，而是复用静态障碍物绕障的目标筛选与普通换道模块的路径生成。官方模块说明对此有明确描述（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md:14-16`），源码中 `AvoidanceByLaneChange` 也直接继承 `NormalLaneChange`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp:36-47`）。

对简化模块最重要的结论是：

1. 应复用原生的“有效路径是执行请求的前置条件”这一状态原则，避免不可行目标进入 `RUNNING` 后占位；不必照搬完整取消/中止状态机。
2. 应复用按 RouteHandler 拓扑和 lanelet ID 关联当前道/目标道的方法，不能按两个序列的数组下标配对。
3. 至少应增加目标邻道占用检查和“无安全路径则停车/等待”；完整 RSS、多预测轨迹、abort path 可作为简化边界之外的能力。
4. 应以 `previous_module_output` 为输出基底，合并 drivable-area 信息，并显式生成换道灯；这既简单又避免破坏上游元数据。
5. 原生绕障也只按障碍物所在侧选择一个方向，没有自动尝试反方向。因此“备用方向”不是可直接照抄的原生机制；若要增加，必须把它定义为 AGV 特有策略，并对两个方向分别做拓扑和安全判定。

## 1. 请求、运行、成功与失败状态

### 1.1 原生模块如何避免不可行目标占住模块

PlannerManager 在判断 idle 模块是否请求执行前，会先设置 PlannerData、上游输出并调用 `updateData()`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:62-68`）。原生换车道绕障的 `updateData()` 继承自 `LaneChangeInterface`：它更新车道、目标物、瞬态数据和绕障特有数据，并在等待批准时生成/更新换道状态（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/interface.cpp:101-115`）。

换车道绕障随后只有在以下三个条件同时成立时才请求执行：

- 普通换道前提成立；
- 绕障特有触发条件成立；
- 已生成有效路径。

对应判断是 `isLaneChangeRequired() && specialRequiredCheck() && isValidPath()`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/interface.cpp:48-52`）。其中普通换道若数据不足、离目标道太远、靠近管制要素等，会返回不请求执行（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:294-325`）；目标道不存在时 `getSafePath()` 直接返回 invalid（同文件 `:261-281`）。因此，持续存在的障碍物如果没有可用邻道或无法生成有效路径，不会被注册为正在运行的模块，也就不会以“仅透传路径但一直 RUNNING”的形式占住 slot。

这里必须区分“有效但不安全”和“根本无有效路径”：`getSafePath()` 会保留首条有效但不安全的 candidate，同时将 `is_valid_path=true`、`is_safe=false`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:270-291`）。前者可以进入等待批准/RTC 展示，但 `isExecutionReady()` 为 false；后者根本不请求执行。`isExecutionReady()` 的定义正是“安全且不处于 abort”（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/interface.cpp:96-99`）。

### 1.2 等待批准、运行、成功和失败

通用 SceneModule 状态机定义 `RUNNING`、`WAITING_APPROVAL`、`SUCCESS`、`FAILURE`，并优先检查成功、失败，再处理 RTC 导致的等待/运行转换（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:357-420`）。模块一旦为 `RUNNING`，通用接口明确规定它会持续运行，直到状态转换（同文件 `:136-149`）。

原生换车道绕障进入时调用 `waitApproval()`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/interface.cpp:54-57`）。等待期间若绕障触发条件消失，`specialExpiredCheck()` 是 `!specialRequiredCheck()`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp:93-139`），`canTransitSuccessState()` 会在等待批准时立即成功退出；批准运行后则不会因为目标瞬时消失而立刻撤销，而是继续到换道完成（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/interface.cpp:239-265`）。

运行中若路径失效，接口将其判为 Cancel/Failure（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/interface.cpp:339-367`）；若批准路径动态变得不安全，则依据车辆所处阶段选择 cancel、abort 或 stop，并带迟滞避免抖动（同文件 `:373-410`；`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:1553-1573`）。PlannerManager 会从候选池删除 `SUCCESS`/`FAILURE` 模块（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:731-755`），也会从 approved 链中删除失败模块并丢弃其无效输出（同文件 `:880-905`）。

### 1.3 对简化模块的修复含义

简化模块目前只要 `detectTarget()` 有值就请求执行，并且一旦 `RUNNING` 永远返回 true（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_simple_lane_change_avoidance_module/src/scene.cpp:137-143`）；无邻道/距离不足时却只透传（同文件 `:425-446`）。应将“能否开始 maneuver”前移为请求门槛：idle/候选阶段只有在目标、邻道、距离和路径生成均可行时才返回 execution requested。若已经运行，则保留当前 shift 直到安全回正或明确失败，不能简单因新一帧不可行而瞬间退出。

不建议为封闭低速 AGV 照搬原生完整的 Cancel/Abort/Stop 内部状态机；建议保留三类可验证结果即可：`not_requested`（不可行，不占位）、`running`（已有已锁定 maneuver）、`success/failure`（回正完成或已运行路径失效）。

## 2. 邻道选择、备用方向与地图拓扑

### 2.1 原生方向选择

原生绕障依据最近目标的位置选择相反侧：目标在右则向左换，反之向右（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp:141-154`）。目标选择还包括类别参数、停止时间、包络、横向余量和是否确实需要绕行，而不只是速度阈值（同文件 `:187-299`）。

目标车道选择委托普通 lane-change：非 mandatory 换道严格按已经确定的 `direction` 查询 routing graph 的 `left()` 或 `right()`，并限制不能朝远离 preferred lane 的方向换道（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp:370-399`）。源码没有“首选侧不存在便尝试另一侧”的 fallback。因此，备用方向不能声称是原生行为。

### 2.2 原生如何按拓扑关联车道

原生先通过 RouteHandler 找到目标 lanelet，再从该 lanelet 获取目标 lanelet sequence（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:759-786`）。构造 drivable lanes 时，对每个 current lanelet 查询真实 left/right lanelet，再用 lanelet ID 在目标序列中向前匹配（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp:185-226`）。lane IDs 的组合也从 routing graph 的左右关系和 ID 匹配生成，而不是假设序列等长或同下标相邻（同文件 `:243-292`）。

相比之下，简化模块当前 `generateDrivableLanesWithAdjacent()` 用相同下标配对两个序列（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_simple_lane_change_avoidance_module/src/scene.cpp:70-103`），在分叉、合流或序列起点不同的地图上会误配。此处适合直接复用 `utils::lane_change::generateDrivableLanes()`，或复制其很小的“RouteHandler 邻接查询 + lanelet ID 匹配”原则。

### 2.3 是否增加备用方向

封闭 AGV 场景可以定义“若首选侧不可用，再评估另一侧”，但它是新增产品策略而非修复成原生一致。若采用，必须满足：

1. 两个方向分别通过 RouteHandler 找到真实 route lanelet；
2. 分别计算 shift/path 可行性；
3. 分别检查目标邻道安全；
4. 用确定性规则选择，例如首选绕离障碍物一侧，其次才是备用侧。

当前简化模块没有邻道动态安全检查，因此现在增加 fallback 会扩大进入未经检查邻道的概率，不宜单独实施。

## 3. 安全检查与停车/等待

原生普通换道会生成多组 prepare/lane-changing candidate，并对每条 candidate 做安全检查；找到第一条安全路径即返回，否则保留一条有效但不安全路径用于候选展示（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:1242-1359`）。检查至少覆盖：

- 转弯车道可能出现的超车目标；
- 目标道停车对象导致的延迟换道；
- 车辆 footprint 是否越过目标车道边界；
- ego 与目标物预测轨迹的 RSS 风格碰撞检查；
- ego 卡住时使用另一组安全参数。

对应入口见 `check_candidate_path_safety()`（同文件 `:1362-1412`）。批准后仍会重新检查 overtaking object、delay lane change 和预测轨迹，并用 abort 参数计算安全性（同文件 `:1500-1551`），不是“一次检查后永久信任”。

等待批准或无安全/有效路径时，原生输出当前/terminal 路径并插入停止点（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/interface.cpp:175-210`）。停车位置会考虑当前道末端、可换道边界、前方静止物、目标道阻塞物和 RSS 距离（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:565-656`）。运行中若来自后方的目标使路径不安全且不能 cancel/abort，还存在 Stop state，在输出路径上按制动距离设停止点；源码同时注明该功能当前并不完善（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/interface.cpp:327-336`；同模块 `src/scene.cpp:486-495`）。

对封闭低速 AGV，适合复用的最小安全集合是：

- path footprint 必须处于 current+adjacent 的可行驶区域；
- 换入/借用邻道范围内不得有静止或移动目标占用；
- 从当前速度到 shift end 的纵向距离必须足够；
- 不安全或不可行时，在障碍物前按可配置余量停车并等待，而不是继续以原速度透传上游路径。

完整的多预测轨迹采样、RSS 参数族、intersection/turn-lane 特例、动态 cancel 和 abort path 会显著扩大模块复杂度；若运行环境确实保证封闭、低速、单向且调度层保证邻道清空，可以不照搬，但这些环境保证应成为配置前提和测试假设，而不能默认为天然成立。

## 4. Route 变化与路径连续性

新 route UUID 到达时，BehaviorPathPlanner 节点会整体 reset PlannerManager、当前 route lanelet 和上次 modified goal；注释明确说这样各模块无需自行处理 route jump（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp:361-376`）。PlannerManager 的 reset 会 reset 每个 module manager、当前 route lanelet、处理时间和所有 slots（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/planner_manager.hpp:420-430`）。因此，简化模块不需要维护跨 UUID 的 shift 连续性；route reset 应清空全部 maneuver 缓存。

同一路线运行期间，原生换道在批准后不再重选 current/target lanes（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:75-79`），而是保留已选定的 `status_.lane_change_path` 输出，并仅向目标道前方延伸路径（同文件 `:464-511`）。换道完成也同时检查换道终点、目标道 polygon、航向和横向偏差（同文件 `:788-830`）。这构成一种“批准后冻结 maneuver 几何，直到完成”的连续性策略。

简化模块适合采用同样原则：初始化 maneuver 时缓存完整参考路径、shift lines 和目标邻道；运行中不要每帧用已经裁剪的上游输出重新构造 shift 起点。当前 `extendBackwardLength(original_path)` 内部又把 `getPreviousModuleOutput().path` 当作 `prev_reference`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_simple_lane_change_avoidance_module/src/scene.cpp:512-549`），但调用者传入的 `original_path` 本身就是同一个上游 path（同文件 `:154-163`），所以它没有独立历史源，无法真正恢复已裁掉的起点。

建议保存“maneuver 初始化时的未裁剪 reference path”作为模块成员；后续只以当前 ego 位置裁剪输出，不以每帧上游短路径重建 shift geometry。route reset/processOnExit 时清空该缓存。

## 5. Drivable area、转向灯与 modified goal 输出合并

`BehaviorModuleOutput` 不只有路径，还包含 reference path、turn signal、modified goal 和 drivable-area info（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/data_manager.hpp:121-138`）。后级模块如果从空对象构造输出，等于覆盖上游模块在这些字段中的决定。

原生 lane-change 的做法是先 `auto output = prev_module_output_`，再只替换自己的 path/reference path（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:464-485`）。因此 `modified_goal` 等未由它负责的字段自然保留。随后它生成 current+target drivable lanes，并用 `combineDrivableAreaInfo(current, previous)` 合并上游信息（同文件 `:514-531`）。合并函数不仅合并 lane，还追加两边的 obstacle，并 OR 各类 expansion flag（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/utils/drivable_area_expansion/static_drivable_area.cpp:1776-1812`）。

运行中的 lane-change 会根据换道方向生成 turn signal，再通过 `TurnSignalDecider::overwrite_turn_signal()` 与上游信号仲裁（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp:500-509`）。等待批准时，绕障类型暂时沿用上游 turn signal；普通换道才提前产生自己的信号（同文件 `:352-382`，以及 interface `src/interface.cpp:175-181`）。

简化模块目前在 `adjustDrivableArea()` 中新建空 `BehaviorModuleOutput`，只设置 path、reference path 和新的 drivable lanes（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_simple_lane_change_avoidance_module/src/scene.cpp:359-394`）。应改为：

1. 从 `getPreviousModuleOutput()` 拷贝；
2. 替换 path/reference path；
3. 用 `combineDrivableAreaInfo()` 合并新的车道与上游 drivable area，保留 obstacles/flags；
4. 基于实际方向与 shift start/end 生成换道灯，并用 turn signal decider 仲裁；
5. 不触碰 `modified_goal`。

这是低复杂度、高收益、与 AGV 简化目标不冲突的直接复用。

## 6. Candidate、approval 与 RTC

原生换道把 candidate 和实际输出分开：`planCandidate()` 返回候选路径、横移量、距 path change 起终点的距离（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp:317-328`）；`planWaitingApproval()` 仍输出可安全行驶/停车的当前路径，同时单独公布 candidate（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/interface.cpp:175-210`）。

RTC 状态包含方向、ready/safe、等待或运行状态以及 maneuver 起终点距离；换车道绕障按左右 RTC channel 更新（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/interface.cpp:59-71`）。RTC 未批准时 SceneModule 会保持 `WAITING_APPROVAL`，批准后转 `RUNNING`（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:314-354`）。成功时通用接口发送 `SUCCEEDED`（同文件 `:547-558`）。PlannerManager 在多个 candidate 中按优先级和 simultaneous-executable 配置筛选，并优先选择已经批准的模块（`src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:671-799`）。

封闭低速 AGV 如果没有人工/远程批准流程，不必为了“像原生”而强制引入 RTC 等待，否则会增加系统依赖和状态分支。仍建议保留正确的 CandidateOutput，用于调试、可视化和未来接 RTC；自动执行时则把“execution ready”绑定到最小安全检查。若现场需要调度系统确认借道，原生 candidate/RTC 分离非常适合复用：未批准输出停车路径，候选路径只作为审批信息，批准后才应用。

## 7. 修复优先级：复用什么，舍弃什么

### 第一优先级：直接复用，不违背简化目标

1. **执行请求前置可行性**：目标 + 真实邻道 + 足够距离 + 可生成路径，任一失败都不进入 RUNNING。
2. **拓扑匹配**：RouteHandler 邻接关系 + lanelet ID 匹配，替换下标配对。
3. **输出基底与合并**：复制 previous output，合并 DrivableAreaInfo，保留 modified goal/obstacles/flags。
4. **换道灯**：根据方向生成并与上游 turn signal 仲裁。
5. **maneuver 冻结与历史 reference path**：开始后缓存几何，直到回正/失败；route reset 清空。
6. **不可行停车**：至少保证车辆能在目标前停车等待，而不是无条件透传速度。

### 第二优先级：按封闭 AGV 假设做精简版

1. 邻道当前占用与短时预测检查；无需一开始就实现完整对象分类和所有预测轨迹组合。
2. footprint 位于 current+adjacent drivable area 的检查。
3. 运行期安全复查和有限迟滞；若变得不安全，在尚未横移时停车/取消，已横移时继续受控完成或停车，不必立刻实现复杂 abort path。
4. 可选的备用方向策略；只有在两个方向都独立做完拓扑与安全检查后才启用。

### 暂不复用：会显著突破简化目标

1. 完整 RSS 参数族、所有 predicted path 组合和多级对象过滤策略；
2. intersection、turn lane、traffic light、no-lane-change marking 的全套道路规则；
3. 高速道路需要的 cancel/abort trajectory 生成与二次批准；
4. 没有外部审批需求时的完整 RTC 生命周期；
5. 普通道路多阶段速度/加速度/frenet/path-shifter 大规模采样。

“暂不复用”成立的前提是模块用途仍被严格限定为封闭、低速、受调度的 AGV 环境。一旦要进入开放道路或允许邻道动态交通，这些机制就不再是可选复杂度，而是安全功能。

## 建议的实现顺序

1. 先修输出保留/合并和拓扑匹配；两者局部、可测试且不会改变产品策略。
2. 把可行性计算提取为纯结果结构，在 `isExecutionRequested()` 与 `plan()` 共用；修复不可行目标占位。
3. 增加 maneuver reference path 缓存和跨周期连续性测试。
4. 增加最小邻道安全检查与障碍物前停车。
5. 最后决定是否需要备用方向和 RTC；这两项属于策略选择，不应混在基础正确性修复中。
