# Simple Avoidance distance and trajectory test

先启动仿真（建议关闭 RViz，使用当前已安装的 `default` planning preset）：

```bash
cd /home/nvidia/autoware
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/nvidia/autoware_map/3_test/ \
  vehicle_model:=byd_vehicle sensor_model:=byd_sensor_kit \
  lanelet2_map_file:=0727_lanelet2_map.osm \
  pointcloud_map_file:=pointcloud_map.pcd \
  launch_didrive_perception:=false scenario_simulation:=false rviz:=false \
  launch_simple_avoidance:=true launch_simple_lc_avoidance:=false \
  launch_obstacle_stop_module:=false launch_dynamic_obstacle_stop_module:=false
```

另一个终端运行：

```bash
cd /home/nvidia/autoware
python3 src/byd/obstacle_avoidance_limit_test/scripts/simple_avoidance_distance_test.py \
  --mode simple_avoidance --record-bag
```

驱动按手工验证过的顺序执行：初始化位姿、设置终点、切换自动模式，
确认里程计有实际前进后才发布障碍物。距离脚本默认使用
`initial_engage_state=true`；若显式设置 `INITIAL_ENGAGE_STATE=false`，车辆
可能保持静止，此类样本会被标记为无效，不能用于绕障距离边界。

For the isolated Simple Lane Change Avoidance run, restart the simulator with
all of the following switches (the four switches are deliberately explicit):

```text
launch_simple_avoidance:=false
launch_simple_lc_avoidance:=true
launch_obstacle_stop_module:=false
launch_dynamic_obstacle_stop_module:=false
```

Then run the same driver with `--mode simple_lc_avoidance`.  The driver uses
the corresponding `path_candidate` and planning-factor topics.  Do not change
only the driver mode while leaving the other path module loaded: both modules
can otherwise consume the same object and the result is not attributable to
the selected module.

结果写入 `log/<日期>/simple_avoidance_distance_test/`，包括：

- `distance_sweep.csv`：距离、侵入量、左右侧、候选路径、最终轨迹和实际轨迹指标；
- `README.md`：最小/最大成功距离、最小失败距离、最大未触发距离和停车接管统计；
- `scenario_*/rosbag2/`：候选路径、最终轨迹、里程计、目标和规划因子。

`OBSTACLE_STOP_TAKEOVER` 不计入纯绕障成功边界。隔离测试启动时同时关闭静态和动态 obstacle-stop，相关因子仍记录用于确认没有接管。脚本的 `speed_mps` 是测试标签，实际速度以 `actual_speed_max_mps` 为准；低速 `0.3 m/s` 需要通过单独的速度限制 preset 或运行时速度参数真正施加，不能仅靠改变标签。

参数敏感性使用 `config/simple_avoidance_sensitivity.yaml` 逐项生成测试矩阵。每次改变一个参数后必须重新启动 planning simulator，避免 ROS 参数只在节点启动时读取造成假对比。

当前判定还要求车辆实际纵向越过障碍物，并用车辆与障碍物在车道坐标系中的保守矩形间隙判定碰撞；因此只看到模块输出侧移而车辆没有驶过障碍物，不会计为成功。

距离边界的主口径是模块第一次输出 `active target locked` 时的
`first_target_lock_lon_m`，即当前自车参考位姿到障碍物中心沿参考路径的纵向弧长。
`longitudinal_m` 仅表示 dummy obstacle 的初始放置距离，不能代替第一次锁定距离。
同时应记录锁定时的 `ego_speed`、`dist_to_shift_end` 和 `dist_to_obstacle`；
后两者分别表示完成准备/侧移所需距离和扣除障碍物半长及安全余量后的可用距离。
