###
 # @Autor: wei.canming
 # @Version: 1.0
 # @Date: 2026-03-24 16:45:56
 # @LastEditors: wei.canming
 # @LastEditTime: 2026-03-25 11:37:42
 # @Description: 
### 
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/root_reference_path --once > root_reference_path.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once > path_with_lane_id.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once > path.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_smoother/path --once > path_smoother.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory --once > path_optimizer.log && \
ros2 topic echo /planning/scenario_planning/lane_driving/trajectory --once > trajectory_lane_driving.log && \
ros2 topic echo /planning/scenario_planning/scenario_selector/trajectory --once > trajectory_scenario_selector.log && \
ros2 topic echo /planning/scenario_planning/velocity_smoother/trajectory --once > trajectory_velocity_smoother.log && \
ros2 topic echo /planning/scenario_planning/trajectory --once > trajectory_final.log

ros2 bag record \
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/root_reference_path \
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id \
/planning/scenario_planning/lane_driving/behavior_planning/path \
/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path \
/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory \
/planning/scenario_planning/lane_driving/trajectory \
/planning/scenario_planning/scenario_selector/trajectory \
/planning/scenario_planning/velocity_smoother/trajectory \
/planning/scenario_planning/trajectory \
-o path_record

ros2 bag record -e "^/planning($|/)" -o planning_topics_bag