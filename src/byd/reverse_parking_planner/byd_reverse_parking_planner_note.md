# 通过服务设置目标位姿并立即触发规划
ros2 service call /reverse_parking_planner/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 12.74484634399414, y: 222.78298950195312, z: 2.7753277658298336}, orientation: {x: 0.0, y: 0.0, z: -0.6680717263004464, w: 0.7440968811370878}}}}"

# 使用已有目标重新规划
ros2 service call /reverse_parking_planner/trigger_planning std_srvs/srv/Trigger