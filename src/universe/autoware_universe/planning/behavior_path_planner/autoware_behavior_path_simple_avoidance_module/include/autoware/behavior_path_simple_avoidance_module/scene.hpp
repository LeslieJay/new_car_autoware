// Copyright 2025 BYD
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__SCENE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_simple_avoidance_module/data_structs.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

class SimpleAvoidanceModule : public SceneModuleInterface
{
public:
  SimpleAvoidanceModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<SimpleAvoidanceParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override { return true; }
  void updateData() override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<SimpleAvoidanceParameters>>(parameters);
  }

private:
  ModuleStatus setInitState() const override { return ModuleStatus::RUNNING; }

  bool canTransitSuccessState() override;
  bool canTransitFailureState() override { return false; }

  void initVariables();
  std::optional<AvoidanceTarget> detectTarget(
    const std::optional<std::string> & preferred_uuid = std::nullopt,
    bool use_hold_hysteresis = false) const;
  std::optional<AvoidanceTarget> updateTargetMetrics(const AvoidanceTarget & target) const;
  std::optional<AvoidanceTarget> getActiveTargetOrHeldTarget();
  bool isEgoOnShiftLine() const;
  NoTargetDiagnosis diagnoseNoTarget() const;
  ShiftLineArray buildShiftLines(const AvoidanceTarget & target, double shift_length) const;
  BehaviorModuleOutput adjustDrivableArea(const ShiftedPath & path) const;
  BehaviorModuleOutput passThrough(
    InfeasibleReason reason, const PassThroughDebugInfo & debug_info = {}) const;
  PathWithLaneId extendBackwardLength(const PathWithLaneId & original_path) const;
  void setDebugMarkersVisualization() const;

  PathWithLaneId reference_path_{};
  lanelet::ConstLanelets current_lanelets_{};
  std::shared_ptr<SimpleAvoidanceParameters> parameters_;
  PathShifter path_shifter_;
  ShiftedPath prev_output_{};
  std::optional<AvoidanceTarget> active_target_;
  mutable SimpleAvoidanceDebugData debug_data_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__SCENE_HPP_
