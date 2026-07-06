# 倒车停车快速命令

## 启动

```bash
ros2 launch byd_launch reverse_parking.launch.py
```

## 设置目标并规划

```bash
ros2 service call /reverse_parking_planner/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 120.16, y: -93.21, z: -0.788}, orientation: {x: -0.000022, y: 0.00010, z: 0.208, w: 0.978}}}}"
```

x: 130.16121427664666
      y: -93.21653002571988
      z: -0.7889642785164361
    orientation:
      x: -2.2900283281494027e-05
      y: 0.00010762539769507308
      z: 0.20811861988795236
      w: 0.9781035875347174


## 重新规划

```bash
ros2 service call /reverse_parking_planner/trigger_planning std_srvs/srv/Trigger
```

## 接口检查（节点运行后）

```bash
ros2 service list | grep reverse_parking_planner
ros2 topic list | grep -E 'external/selected|planning/parking'
```
