// Copyright 2026 BYD
// SPDX-License-Identifier: Apache-2.0
#ifndef AVOIDANCE_BY_LANE_CHANGE__OPPOSITE_LANE_HPP_
#define AVOIDANCE_BY_LANE_CHANGE__OPPOSITE_LANE_HPP_

#include "data_structs.hpp"
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <byd_vehicle_msgs/msg/trailer_configuration.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
enum class BorrowingPhase { WAITING_APPROVAL, BORROWING, PASSING, RETURNING, STOPPING, COMPLETE };

// An independent planner: it never changes NormalLaneChange's lanes or object filters.
class OppositeLaneBorrowing
{
public:
  OppositeLaneBorrowing(
    rclcpp::Node & node, std::shared_ptr<AvoidanceByLCParameters> parameters,
    std::shared_ptr<LaneChangeParameters> lane_change_parameters);
  void update(
    const std::shared_ptr<const PlannerData> & data, const BehaviorModuleOutput & upstream,
    const std::optional<ObjectData> & target, bool prefer_same_direction);
  bool requested() const { return requested_; }
  bool ready() const { return safe_; }
  bool committed() const { return committed_; }
  bool complete() const { return phase_ == BorrowingPhase::COMPLETE; }
  bool left() const { return left_; }
  BorrowingPhase phase() const { return phase_; }
  const std::string & reason() const { return reason_; }
  std::string approvalKey() const;
  friend class OppositeLaneBorrowingTestPeer;
  const std::optional<Pose> & stopPose() const { return stop_pose_; }
  CandidateOutput candidate() const;
  BehaviorModuleOutput output(bool waiting);
  void reset();

private:
  bool buildCorridor();
  bool buildPath();
  bool tryReturn();
  bool validate(const ShiftedPath & path, bool recovery = false);
  bool contained(const autoware_utils::Polygon2d & footprint, bool source_only = false) const;
  std::vector<autoware_utils::Polygon2d> footprints(const ArticulatedPose & pose) const;
  ArticulatedPath articulated(const PathWithLaneId & path, size_t start) const;
  bool updateActualHistory();
  void stop(PathWithLaneId & path, bool waiting);
  void updatePhase();
  void logDiagnostics(std::chrono::steady_clock::time_point started);
  double trainLength() const;
  void configure(const byd_vehicle_msgs::msg::TrailerConfiguration & message);

  std::shared_ptr<AvoidanceByLCParameters> parameters_;
  std::shared_ptr<LaneChangeParameters> lc_parameters_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  rclcpp::Subscription<byd_vehicle_msgs::msg::TrailerConfiguration>::SharedPtr trailer_sub_;
  mutable std::mutex configuration_mutex_;
  ResolvedTrailerConfiguration configuration_;
  bool configuration_received_{false};
  bool configuration_valid_{false};
  size_t configuration_revision_{0};
  size_t active_revision_{0};
  ResolvedTrailerConfiguration active_configuration_;
  std::shared_ptr<const PlannerData> data_;
  BehaviorModuleOutput upstream_;
  std::optional<ObjectData> target_;
  PathWithLaneId reference_;
  ShiftedPath path_;
  ShiftLineArray lines_;
  lanelet::ConstLanelets source_;
  lanelet::ConstLanelets opposite_;
  std::vector<DrivableLanes> drivable_lanes_;
  std::vector<Pose> actual_history_;
  std::vector<Pose> observed_history_;
  std::optional<std::string> route_id_;
  bool context_changed_{false};
  bool requested_{false};
  bool safe_{false};
  bool committed_{false};
  bool left_{false};
  bool recovery_{false};
  BorrowingPhase phase_{BorrowingPhase::WAITING_APPROVAL};
  std::string reason_;
  std::string conflict_object_;
  std::optional<rclcpp::Time> clear_since_;
  bool conflict_seen_{false};
  size_t completion_count_{0};
  std::optional<Pose> stop_pose_;
  std::optional<std::chrono::steady_clock::time_point> deadline_;
  std::vector<double> reference_velocity_limits_;
  std::optional<BorrowingPhase> last_diagnostic_phase_;

};
}  // namespace autoware::behavior_path_planner
#endif  // AVOIDANCE_BY_LANE_CHANGE__OPPOSITE_LANE_HPP_
