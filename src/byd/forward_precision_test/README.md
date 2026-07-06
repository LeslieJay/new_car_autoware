# 正向前进终点精度排查

用于排查 Autoware **纯自动驾驶链路**（`gate_mode=AUTO`, `operation_mode=AUTONOMOUS`）下的终点精度问题。

## 快速流程

```bash
# 1. 发车前自检（不通过则拒绝测试）
bash src/byd/forward_precision_test/verify_forward_mode_baseline.sh
# 或自动修复后重检
bash src/byd/forward_precision_test/verify_forward_mode_baseline.sh --fix

# 2. 录包（内置自检，通过后才录）
bash src/byd/forward_precision_test/record_forward_precision_test.sh 0703_forward_test

# 3. 发布目标点（示例）
ros2 topic pub --once /planning/mission_planning/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 261.444300524526, y: -28.175441045963222, z: 89.18}, orientation: {x: -0.0033794693663022498, y: 0.003372215997922873, z: 0.24660542673218327, w: 0.9691042105224306}}}"

# 4. 到达终点停稳后 Ctrl+C 停止录包

# 5. 分析
python3 src/byd/forward_precision_test/analyze_forward_endpoint_accuracy.py \
  log/0703_forward_test \
  --goal-x 261.444300524526 --goal-y -28.175441045963222
```

## 误差分解

| 指标 | 含义 |
|------|------|
| `planning_goal_to_traj_end` | 规划终点与 mission goal 偏差 |
| `control_traj_end_to_odom` | 实际位置与规划轨迹末点偏差 |
| `total_goal_to_odom` | 总终点误差 |

若 `planning` 误差 < 0.5m 但 `total` > 2m 且 `remaining_distance` 长时间不降，判定为**提前停车**而非规划精度问题。

## 注意事项

- **禁止**在正向前进精度测试中使用 `toggle_control_mode`（该服务切换 AUTO/EXTERNAL 控制来源，不是倒车模式）。
- 倒车流程请使用 `setup_external_control_mode.sh` + `reverse_parking_planner`，与正向 Autoware 控制链路分开。
