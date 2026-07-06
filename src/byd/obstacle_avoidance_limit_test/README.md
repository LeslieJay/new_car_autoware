# 仿真绕障极限测试

在 `planning_simulator` 中通过 Python 脚本向 `/simulation/dummy_perception_publisher/object_info` 发布静态障碍物，从路肩边缘逐步侵入车道，测量 `static_obstacle_avoidance` 的极限距离。

## 前置条件

- 已编译 Autoware 并 `source install/setup.bash`
- 地图路径默认 `/home/nvidia/autoware_map/9_out/`
- 默认 preset 已启用 `launch_static_obstacle_avoidance: true`

## 目录结构

```
obstacle_avoidance_limit_test/
├── config/
│   ├── baseline.yaml   # 固定 ego/goal/障碍物基准
│   └── metrics.yaml    # 判定指标与 sweep 参数
├── scripts/
│   ├── dummy_object_utils.py
│   ├── setup_scenario.py
│   ├── place_obstacle.py
│   └── obstacle_avoidance_sweep.py
├── run_planning_simulator.sh
├── run_smoke_test.sh
└── run_sweep_test.sh
```

## 快速开始

### 1. 启动仿真

```bash
cd /home/nvidia/autoware/src/byd/obstacle_avoidance_limit_test
./run_planning_simulator.sh
```

### 2. Smoke test（单点验证）

另开终端：

```bash
./run_smoke_test.sh 0.4
```

### 3. 横向极限 sweep

```bash
./run_sweep_test.sh
```

结果输出到 `results/<timestamp>/sweep_results.csv` 和 `summary.yaml`。

## 配置说明

### baseline.yaml

- `ego` / `goal`：固定初始位姿与目标点
- `obstacle.lane_yaw` / `lane_width` / `shoulder_side`：车道几何
- `obstacle.longitudinal_distance_m`：障碍物距 ego 的前向距离

**首次使用前请按实际地图微调 `ego.x/y/yaw` 和 `goal`，确保路径经过测试路段。**

### metrics.yaml

- `lateral_sweep`：路肩侵入 sweep 范围（默认 0.0~1.5 m，步长 0.2 m）
- `min_shift_length_m`：判定成功绕障的最小横向偏移（0.09 m）
- `timing`：setup / settle / sample 等待时间

## 障碍物放置逻辑

以 ego 为纵向零点，沿 `lane_yaw` 前进 `longitudinal_distance_m`：

```
侵入 0 m  → 障碍物内侧贴路肩/车道边界
侵入增大 → 障碍物向车道中心移动
```

横向偏移公式（右路肩）：

```
lateral = lane_width/2 - intrusion - obj_width/2
```

## 判定标准

| 结果 | 条件 |
|------|------|
| SUCCESS | `allow_avoidance=true` 且出现 SHIFT 行为或 `max_shift_length >= 0.09m` |
| PARTIAL | 允许绕障但无有效偏移，或仅 STOP 等待 |
| FAIL | 持续无法绕障 / 被过滤 |
| NO_OBJECT | 感知未收到障碍物 |

## 观测 Topic

- `/simulation/dummy_perception_publisher/object_info`
- `/perception/object_recognition/objects`
- `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array`
- `/planning/planning_factors/static_obstacle_avoidance`
- `/planning/path_reference/static_obstacle_avoidance`

## 手动单点放置

```bash
source /home/nvidia/autoware/install/setup.bash
python3 scripts/place_obstacle.py --intrusion 0.6 --clear-first --verify
python3 scripts/place_obstacle.py --intrusion 0.8 --longitudinal 30 --clear-first
```

## 细扫极限

找到粗扫分界后，修改 `metrics.yaml`：

```yaml
lateral_sweep:
  start_m: 0.8
  stop_m: 1.0
  step_m: 0.05
```

再运行 `./run_sweep_test.sh`。
