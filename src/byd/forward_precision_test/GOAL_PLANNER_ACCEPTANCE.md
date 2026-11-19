# 园区 AGV 固定终点规划验收

适用：最高速度 2.0 m/s 的封闭园区 AGV；针对 2026-11-18 15:25 停车问题的路径生成修复。精停局部加密必须保留此前的弯道、lane IDs 和速度限制。

## 配置

Autoware launch 的有效车辆配置为：

|项目|值|作用|
|---|---:|---|
|规划最高速度|2.0 m/s|沿用 common.param.yaml 中的既有限制|
|fixed_goal.request_length|3.0 m|固定终点模块请求范围，之前为 30 m|
|终点连接半径上限|min(既有半径, request_length/2)|3 m 请求对应最多 1.5 m，避免较晚接管却重连更远的上游路径|
|goal_resample_interval|0.05 m|末段真实加密间距；此前通用重采样的 0.2 m 合并容差会吞掉加密点|
|goal_planner.th_arrived_distance|0.1 m|本车辆配置的模块到达距离；不是外部真值精度保证|
|PlanningValidator.default_handling_type|2|无效轨迹采用上一有效轨迹并减速|
|trajectory_shift.handling_type|2|不连续轨迹也采用上述策略|
|soft_stop_deceleration / jerk|-1.0 m/s² / 1.0 m/s³|异常轨迹的受控减速参数|

3 m 是几何接管范围，不是从 2 m/s 开始制动的触发距离。正常进场的减速仍由上游速度规划完成。包内通用默认配置保持不变；上述值来自 autoware_launch 的车辆配置，并应通过运行时参数服务核对。

## 编译与回归测试

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/autoware/install/setup.bash
cd /home/nvidia/autoware
colcon build --packages-select autoware_behavior_path_planner_common \
  autoware_behavior_path_goal_planner_module --symlink-install
colcon test --packages-select autoware_behavior_path_goal_planner_module
```

新增 C++ 测试检查：终点区域之前的弯道点/车道/速度保持；末尾真正按 5 cm 加密；终点连接重采样不提前覆盖上游 lane IDs。原问题在修复前明确触发失败，再执行修复后的原生测试。

## 闭环仿真

```bash
python3 src/byd/forward_precision_test/run_goal_planner_acceptance.py \
  --case turn --output artifacts/goal_acceptance/turn_3m --domain 63
python3 src/byd/forward_precision_test/run_goal_planner_acceptance.py \
  --case straight --output artifacts/goal_acceptance/straight_3m --domain 65
# 仅在隔离测试实例中覆盖成旧值，验证代码修复不依赖较晚接管来掩盖问题。
python3 src/byd/forward_precision_test/run_goal_planner_acceptance.py \
  --case turn --request-length 30 \
  --output artifacts/goal_acceptance/turn_30m_regression --domain 63
```

脚本启动真实 planning_simulator、规划器、控制器和车辆模型，使用 hy_1031 地图。转弯场景的起点约 (-26.054, -13.319)，终点约 (-57.275, 3.816)，经过与故障相关的 1338→1407→907 路段。无障碍环境由自由空间 OccupancyGrid 明确定义，避免空点云时栅格节点不发布地图而使规划永久等待；不注入预制轨迹或控制命令。

验收要求：

- 实际运动并到达目标，在目标 0.10 m 内以速度绝对值<0.02 m/s 持续停稳 2 s，最终航向误差≤5°。
- 最大速度≤2.05 m/s，最大闭环跟踪误差≤0.30 m。
- 固定终点模块实际运行，首次激活不能早于请求范围加 0.5 m 的采样余量。
- 行驶开始后录制的 gate 控制输出最大间隔≤0.5 s，控制输出和验证状态有足够样本。
- PlanningValidator 不出现几何/动力学无效状态；时延状态单独保留，不用于证明几何故障。

控制连续性以录制 bag 为准。在线测试观察节点调用服务期间可能暂停处理订阅，不能把该节点自身的接收空档认定为控制器中断。脚本保留在线间隔，并以独立 recorder 的实际 gate 输出时间重新验收。

这项停车阈值是 10 cm 的路径连续性回归标准，不代表 ±2 cm 的实车精度验收。

## 原始 bag 的输入回放

```bash
python3 src/byd/forward_precision_test/prepare_goal_replay_bag.py \
  /mnt/driving_recorder/20261118/accuracy/143827_bag \
  artifacts/goal_acceptance/replay_inputs
python3 src/byd/forward_precision_test/run_goal_planner_replay.py \
  --bag artifacts/goal_acceptance/replay_inputs \
  --output artifacts/goal_acceptance/replay_3m --domain 64
```

默认窗口：北京时间 2026-11-18 15:23:00～15:27:23。提取只读访问源 SQLite，并补充窗口之前的路线/模式最近记录及初始静态 TF。回放启动真实 logging_simulator 的规划与控制节点，重新生成路径和控制。**不回放旧规划/控制输出**。两种回放都不能替代闭环运动仿真。

开放环验收要求：重新生成轨迹≥500 帧，follower 和 gate 控制各≥1000 条；启动 5 s 后无几何/动力学校验无效状态，控制最大记录间隔≤0.5 s，无横向控制求解失败，固定终点模块确实被执行，最大车到轨迹距离≤0.75 m。

0.75 m 对应既有园区执行位置容差；不能套用闭环跟踪标准到这段记录，因为原车人工移动和错误路径留下的实测偏离仍会被回放。验收报告必须记录最大距离及与原始参考路径的对比，不得将它描述为修复后的闭环跟踪误差。此次最坏样本的原车距录制参考路径约 0.563 m，新生成轨迹距离约 0.635 m，增量约 0.072 m；原先统一使用 0.5 m 的阈值会把这段开放环固有偏离判成失败，该初始结果保留在分析记录中。

## 证据与限制

每次运行保存 commands.json、launch/scenario/record/play 日志、输入清单、输出 bag 和 verdict.json。当前修改的实测结果见 `/home/nvidia/autoware/artifacts/20261118_goal_fix/`。

仿真和回放期间观察到独立的 manual_lane_change_handler 在接收路线后崩溃。首轮公共库修复生效前已有同样表现；本次没有修改该节点。上述验收覆盖自动固定终点规划→控制→车辆模型链路，不能据此宣称包括手动变道在内的整个 Autoware 功能均通过验收。
