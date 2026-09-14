# Simple Lane Change Avoidance

该模块面向封闭、低速场地中的借道绕障。它只在当前车道和相邻 route
车道的 union 内生成轨迹；轨迹必须同时通过车辆 footprint、感知目标碰撞、
挂车 footprint 和铰接角检查。检查失败、邻道被占用、距离不足或目标在未
通过前丢失时，模块输出目标前安全停车路径并发布 planning factor。

`max_shift_length` 是硬安全上限，默认 `4.5 m`；需求位移超过该值会返回
`no_room`，不会被静默截断。路径生成临时失败时最多复用上一条已验证且连续
的路径 `path_generation_failure_timeout` 秒，随后停车。

目标以 UUID 优先关联，短时丢失使用 `target_lost_time_threshold` 内的 held
target；超过阈值且尚未通过时停车。通过后必须连续满足
`completion_stable_count` 个周期才退出模块。换道和回原车道的转向灯由
planner turn-signal decider 生成，模块完成或停车后恢复上游输出。

## 挂车

挂车配置通过 `/vehicle/status/trailer_configuration`
（`byd_vehicle_msgs/msg/TrailerConfiguration`）发布。消息中的类型名必须在
`trailer.type_names` 中声明，并且只能在车辆静止时修改。牵引车、全部挂车
按 `trailer.footprint_sampling_interval` 重建 footprint；超宽、碰撞或铰接
角超限分别导致安全停车。

## 隔离仿真验收

关闭另一个绕障模块和两个 obstacle-stop 模块后运行：

```bash
cd /home/nvidia/autoware
BASE_DOMAIN=110 ./src/byd/obstacle_avoidance_limit_test/run_simple_lc_avoidance_acceptance.sh
```

地图默认使用 `/home/nvidia/autoware_map/3_test/0727_lanelet2_map.osm` 和
BYD vehicle/sensor model。可用 `MAP_PATH`、`LANELET2_MAP_FILE`、
`POINTCLOUD_MAP_FILE`、`OUT`、`SAMPLE_SEC` 覆盖。每个场景独立启动
`planning_simulator`、分配 ROS domain、记录 rosbag，并在输出目录生成：

- `acceptance_results.csv`：逐场景机器结果；
- `acceptance_summary.json`：参数、结果及门禁状态；
- `acceptance_summary.md`：人工可读汇总；
- `scenario_*/rosbag2/`：路径、轨迹、里程计、目标、挂车配置、速度限制和 planning factor。

允许结果只有 `SUCCESS` 和预期 `SAFE_STOP`；`INVALID`、`COLLISION`、
`TRACKING_FAILURE` 或空路径都会使脚本以非零退出码结束。
