# static_obstacle_avoidance / avoidance_by_lane_change 闭环对照

使用 `hy_1031`、BYD 无挂车车辆，保留普通换道、goal planner 及下游安全停车功能。
仅切换两个目标模块的启动开关：A 仅 static，B 仅 lane-change avoidance，C 双开。
不覆盖现有生产参数，不修改模块算法或 RTC 配置，不向车辆最终控制接口发送强制行驶命令。

## 使用

在工作区根目录运行：

```bash
src/byd/obstacle_avoidance_limit_test/run_dual_avoidance_test.sh --prepare-only --output /tmp/dual-map-check
src/byd/obstacle_avoidance_limit_test/run_dual_avoidance_test.sh --cases empty --groups A,B,C --repeats 1
src/byd/obstacle_avoidance_limit_test/run_dual_avoidance_test.sh
```

最后一条执行全部适用场景，每组重复三次；需要数小时。每个用例重新启动仿真，使用私有 ROS domain，
启动前检查占用并锁定；不停止已有的用户节点。默认起始 domain 为 174，可用 `--domain` 改变。
通信后端默认沿用工作区的 `RMW_IMPLEMENTATION`（本机为 Cyclone DDS），可用 `--rmw` 显式选择。
后端写入批次清单和启动证据，续跑沿用批次记录；不能在同一批次混用不同后端。
Cyclone 在 localhost-only 模式下需要容纳整套仿真的数十个进程。未设置 `CYCLONEDDS_URI` 时，
入口自动生成批次专用 `cyclonedds.xml`，把自动参与者索引范围扩至 120，并使用 localhost peers；
不改变全局配置。2026-11-21 的实际回归中，默认配置仅 9/20 进程启动，专用配置为 20/20。
可通过 `test/check_cyclone_participants.py --profile /absolute/batch/cyclonedds.xml` 重现容量检查。
无障碍基线失败时停止批量测试，避免把环境问题扩散成模块失败。
启动前还会等待 1 分钟系统负载降至 CPU 核数的 1.5 倍以下，避免多个聊天同时启动仿真时互相干扰。
等待状态写入 `runner_state.json` 和报告；`--max-load` 可调整阈值。它不能保证运行途中没有资源竞争，
所以每个用例同时保存启动和结束时的系统负载。测试中没有关闭延迟检查。

场景、路段、速度、地图在 `config/dual_avoidance.yaml`。位置基于地图中心线弧长生成，
横向值为左正；侵入量是物体**靠车道中心一侧边缘**进入车道的距离。
起终点使用地图高程及切线方向，不沿用旧测试中的坐标。
`--cases empty,right_shallow,center --groups A,B,C --repeats 3` 可以缩小批次。

默认输出为 `log/<北京时间日期>/dual_avoidance_<时间>/`。断点续跑和刷新报告：

```bash
src/byd/obstacle_avoidance_limit_test/run_dual_avoidance_test.sh --resume --output /absolute/result/directory
src/byd/obstacle_avoidance_limit_test/run_dual_avoidance_test.sh --report-only --output /absolute/result/directory
```

续跑跳过已有 verdict 的用例，包括失败用例。修改实现或环境后，用新输出目录复测，保留原始证据。
不要在仿真运行时修改已加载配置；这会破坏三组对照。
未产生 verdict 的中断用例在续跑时移入 `interrupted/` 后重新运行；原 bag 不会被覆盖。
批次运行使用保存的脚本副本，并校验配置、地图及模块二进制哈希；输入变化后停止，不能混合版本。

边界扫描应在代表性场景之后执行；先粗扫，再根据 CSV 中的行为转折点选择细扫区间：

```bash
src/byd/obstacle_avoidance_limit_test/run_dual_avoidance_test.sh --sweep coarse
src/byd/obstacle_avoidance_limit_test/run_dual_avoidance_test.sh --sweep fine --sweep-start 0.8 --sweep-stop 1.0
```

## 证据与判定

每个用例保存 `case.json`、`launch.json`、运行参数、`timeline.jsonl`、sqlite3 rosbag、
实际轨迹与速度图、`actual.json` 和 `verdict.json`；批次目录生成 `summary.csv` 和 `report.md`。
报告生成可以在批次运行中重复调用。没有结果的用例始终标记 PENDING。

- 请求：RTC 的 `requested`、`safe` 和 UUID；RTC 未发布时表示不可观测，不能推断未请求。
- 候选：模块独立 candidate topic，以及 manager candidate 列表。
- 执行：manager 的 **approved + RUNNING**；不由偏移量、跨线或 planning factor 单独推断。
- 拒绝：static 对象过滤信息、RTC 安全状态、对向借道日志与下游停车因素分别记录。
- 安全：实测车身轮廓与注入物体运动模型之间的净距、地图道路包络检查、录制的规划验证器状态。
- 完成：路线到达、实际车身完整越过仍存在的前方障碍物；进入对向车道后必须完整回归。

PASS 需要完整到达与关键证据齐全；SAFE_STOP 表示有原因且安全停车，**不表示恢复能力已通过**。
有释放事件的场景必须在释放后到达，否则区分控制链未就绪与无法恢复。
环境失败、无法到达、缺少 bag 或状态样本均不会算 PASS。出现碰撞或越界时提前终止当前用例。
全部规划验证器失效字段（包括延迟）均保留，不通过屏蔽延迟来获得通过结果。
仅延迟检查失效时记为 ENV_INVALID，不能作为功能通过或模块能力边界；其他验证器失效仍记为异常。
ENV_INVALID 最多自动重试两次，每次重新检查资源，使用新 ROS domain；全部原始证据移入 `retry_attempts/`。
定位环境问题时用 `--cases empty --groups C --repeats 1 --environment-retries 0`，先验证单个基线。
短时检查可 source ROS 环境并设置同一 domain/RMW 后运行 `scripts/probe_dual_environment.py --seconds 10`；
它只订阅消息，不访问正在写入的 SQLite 录包。两种 QoS 均要求 >=5 Hz 且消息年龄 <0.5 秒才通过。
Cyclone 批次的探针还必须设置 `CYCLONEDDS_URI=/absolute/batch/cyclonedds.xml`，与被测进程使用相同配置。
初始位姿在首帧里程计到来前重发，避免监控节点先订阅、仿真器晚订阅而漏收；路线建立后不重置位姿。

当前自动道路检查针对道路物理包络，使用至多 1 mm 简化误差的边界和 0.1 m 车身边缘采样；
它不能替代地图交通规则或各模块安全裕度的逐项审核。此限制在结果报告中明确标注。
物体净距来自注入运动模型，感知跟踪和预测轨迹保存在 bag 里作为独立证据。
曲线场景的通过位置检查保守使用初始航向投影，复杂曲线应结合轨迹图审核。

## 地图覆盖

`map_inventory.json` 保存地图 SHA256、车道中心线、边界标签和共享边界的方向关系。
`hy_1031` 的长直道路段 906/907 为对向共享虚线车道，支持车道内绕障和对向借道对照。
地图不存在同向共享边界车道，所以同向左/右换道、同向交通冲突列为不适用。
未建立可验证的交叉口场景，保留覆盖缺口，不用弯道冒充交叉口，也不静默修改地图。
2026-11-21 观察到 `manual_lane_change_handler` 段错误。固定路线场景不调用这个手动路由接口，
其退出保留在 verdict/报告中，不能将本测试的 PASS 理解为该接口或整套系统健康；其他节点退出记为环境无效。

## 验证测试框架

```bash
python3 -m unittest discover -s src/byd/obstacle_avoidance_limit_test/test -p test_dual_avoidance.py -v
```

测试覆盖候选/执行区分、缺证据禁止通过、碰撞优先、对向回归要求、停车原因要求、
旋转车身距离、非凸区域及地图范围校验。实际车辆闭环仍须通过上述仿真基线验证。
