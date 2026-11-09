// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "interface.hpp"

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::State;
using autoware::route_handler::Direction;

AvoidanceByLaneChangeInterface::AvoidanceByLaneChangeInterface(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters,
  const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: LaneChangeInterface{
    name,
    node,
    parameters,
    rtc_interface_ptr_map,
    objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface,
    std::make_unique<AvoidanceByLaneChange>(parameters, avoidance_by_lane_change_parameters)}
{
  opposite_ = std::make_unique<OppositeLaneBorrowing>(
    node, avoidance_by_lane_change_parameters, parameters);
}

bool AvoidanceByLaneChangeInterface::isExecutionRequested() const
{
  if (opposite_selected_) return opposite_->requested();
  if (getCurrentStatus() == ModuleStatus::RUNNING) return true;
  // isLaneChangeRequired returns an error, not a positive request flag.
  return !module_type_->isLaneChangeRequired().has_value() &&
         module_type_->specialRequiredCheck() && module_type_->isValidPath();
}

void AvoidanceByLaneChangeInterface::processOnEntry()
{
  waitApproval();
}

void AvoidanceByLaneChangeInterface::updateRTCStatus(
  const double start_distance, const double finish_distance)
{
  const auto direction = std::invoke([&]() -> std::string {
    if (opposite_selected_) return opposite_->left() ? "left" : "right";
    const auto dir = module_type_->getDirection();
    return (dir == Direction::LEFT) ? "left" : "right";
  });

  const auto state = isWaitingApproval() && !(opposite_selected_ && opposite_->committed())
    ? State::WAITING_FOR_EXECUTION : State::RUNNING;

  rtc_interface_ptr_map_.at(direction)->updateCooperateStatus(
    uuid_map_.at(direction), isExecutionReady(), state, start_distance, finish_distance,
    clock_->now());
}
void AvoidanceByLaneChangeInterface::updateData()
{
  const bool same_direction_running = getCurrentStatus() == ModuleStatus::RUNNING &&
                                      !opposite_selected_;
  if (same_direction_running) {
    LaneChangeInterface::updateData();
    return;
  }
  auto * avoidance = static_cast<AvoidanceByLaneChange *>(module_type_.get());
  if (!opposite_->committed()) {
    module_type_->setPreviousModuleOutput(getPreviousModuleOutput());
    // Detect from the upstream route before asking NormalLaneChange for a target lane.
    module_type_->updateSpecialData();
    avoidance->refreshUnapprovedLaneChangeData();
    module_type_->resetStopPose();
    updateDebugMarker();
  }
  const bool same_direction_safe = !opposite_->committed() &&
    !module_type_->isLaneChangeRequired().has_value() &&
    module_type_->specialRequiredCheck() && module_type_->isValidPath() && module_type_->isSafe();
  opposite_->update(planner_data_, getPreviousModuleOutput(),
    avoidance->nearestAvoidanceTarget(), same_direction_safe);
  opposite_selected_ = opposite_->requested() || opposite_->committed();
  const auto key = opposite_selected_ ? opposite_->approvalKey() : std::string{"same_direction"};
  if (!opposite_->committed() && key != approval_key_) {
    // Approval for a different target, route, trailer train or branch is not transferable.
    for (const auto & [direction, rtc] : rtc_interface_ptr_map_) {
      if (rtc) rtc->removeCooperateStatus(uuid_map_.at(direction));
      uuid_map_[direction] = generate_uuid();
    }
    approval_key_ = key;
    waitApproval();
  }
  if (opposite_selected_ && (opposite_->committed() || opposite_->ready())) {
    const auto & direction = opposite_->left() ? "left" : "right";
    if (rtc_interface_ptr_map_.at(direction)->isActivated(uuid_map_.at(direction))) {
      clearWaitingApproval();
    }
  }
  if (opposite_selected_) {
    debug_marker_.markers.clear();
    const auto candidate = opposite_->candidate();
    visualization_msgs::msg::Marker marker;
    marker.header = getPreviousModuleOutput().path.header;
    marker.ns = "opposite_borrowing";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.12;
    marker.color.a = 1.0;
    marker.color.g = opposite_->ready() ? 1.0 : 0.0;
    marker.color.r = opposite_->ready() ? 0.0 : 1.0;
    for (const auto & point : candidate.path_candidate.points) {
      marker.points.push_back(point.point.pose.position);
    }
    debug_marker_.markers.push_back(marker);
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.points.clear();
    marker.scale.z = 0.5;
    marker.pose = planner_data_->self_odometry->pose.pose;
    marker.pose.position.z += 2.0;
    marker.text = "opposite borrowing: " + opposite_->reason();
    debug_marker_.markers.push_back(marker);
  }
}

bool AvoidanceByLaneChangeInterface::isExecutionReady() const
{
  // A committed borrowing maneuver retains ownership while its output brakes on safety failures.
  return opposite_selected_ ? (opposite_->committed() || opposite_->ready())
                            : LaneChangeInterface::isExecutionReady();
}

void AvoidanceByLaneChangeInterface::postProcess()
{
  if (!opposite_selected_) LaneChangeInterface::postProcess();
}

BehaviorModuleOutput AvoidanceByLaneChangeInterface::planOpposite(bool waiting)
{
  const auto candidate = opposite_->candidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  // Report safety on the borrowed side for the entire outgoing and returning maneuver.
  const auto & direction = opposite_->left() ? "left" : "right";
  rtc_interface_ptr_map_.at(direction)->updateCooperateStatus(
    uuid_map_.at(direction), opposite_->committed() || opposite_->ready(),
    waiting && !opposite_->committed() ? State::WAITING_FOR_EXECUTION : State::RUNNING,
    candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change,
    clock_->now());
  auto output = opposite_->output(waiting);
  path_reference_ = std::make_shared<PathWithLaneId>(output.reference_path);
  stop_pose_.reset();
  if (opposite_->stopPose()) {
    stop_pose_.emplace(*opposite_->stopPose(), opposite_->reason());
  }
  if (!waiting && opposite_->committed()) lockNewModuleLaunch();
  set_longitudinal_planning_factor(output.path);
  return output;
}

BehaviorModuleOutput AvoidanceByLaneChangeInterface::plan()
{
  return opposite_selected_ ? planOpposite(false) : LaneChangeInterface::plan();
}

BehaviorModuleOutput AvoidanceByLaneChangeInterface::planWaitingApproval()
{
  return opposite_selected_ ? planOpposite(true) : LaneChangeInterface::planWaitingApproval();
}

CandidateOutput AvoidanceByLaneChangeInterface::planCandidate() const
{
  return opposite_selected_ ? opposite_->candidate() : LaneChangeInterface::planCandidate();
}

bool AvoidanceByLaneChangeInterface::canTransitSuccessState()
{
  if (!opposite_selected_) return LaneChangeInterface::canTransitSuccessState();
  return opposite_->complete() || (!opposite_->requested() && !opposite_->committed());
}

bool AvoidanceByLaneChangeInterface::canTransitFailureState()
{
  // A safety failure brakes on the committed geometry; it must not release a borrowed lane.
  return opposite_selected_ ? false : LaneChangeInterface::canTransitFailureState();
}

bool AvoidanceByLaneChangeInterface::isCurrentRouteLaneletToBeReset() const
{
  return !opposite_selected_ && LaneChangeInterface::isCurrentRouteLaneletToBeReset();
}

MarkerArray AvoidanceByLaneChangeInterface::getModuleVirtualWall()
{
  if (!opposite_selected_) return LaneChangeInterface::getModuleVirtualWall();
  // The generic scene interface publishes the longitudinal stop factor from stop_pose_.
  return MarkerArray{};
}

void AvoidanceByLaneChangeInterface::processOnExit()
{
  opposite_->reset();
  opposite_selected_ = false;
  approval_key_.clear();
  LaneChangeInterface::processOnExit();
}

}  // namespace autoware::behavior_path_planner
