// Copyright 2026 BYD
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

#include "autoware/behavior_path_simple_lane_change_avoidance_module/scene.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
namespace
{
BehaviorModuleOutput makeStraightOutput()
{
  BehaviorModuleOutput output;
  for (size_t i = 0; i < 4; ++i) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i);
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 1.0;
    output.path.points.push_back(point);
  }
  output.path.header.frame_id = "map";
  output.reference_path = output.path;
  return output;
}

autoware_perception_msgs::msg::PredictedObjects::SharedPtr makeStaticObstacle()
{
  auto objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  objects->objects.push_back(object);
  return objects;
}

autoware_planning_msgs::msg::LaneletRoute loadLaneChangeRoute()
{
  const auto route_path = autoware::test_utils::get_absolute_path_to_route(
    "autoware_route_handler", "lane_change_test_route.yaml");
  const auto route = autoware::test_utils::parse<autoware_planning_msgs::msg::LaneletRoute>(
    YAML::LoadFile(route_path));
  return route;
}

std::shared_ptr<autoware::route_handler::RouteHandler> makeRouteHandler(
  const autoware_planning_msgs::msg::LaneletRoute & route)
{
  const auto map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
    "autoware_test_utils", "2km_test.osm");
  const auto map_bin = autoware::test_utils::make_map_bin_msg(map_path, 1.0);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin);
  route_handler->setRoute(route);
  return route_handler;
}

autoware_perception_msgs::msg::PredictedObjects::SharedPtr makeObstacleAt(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & path_point)
{
  auto objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose = path_point.point.pose;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  objects->objects.push_back(object);
  return objects;
}

struct NoRoomReengagementScenario
{
  explicit NoRoomReengagementScenario(const std::string & node_name)
  : node(node_name),
    parameters(std::make_shared<SimpleLCAvoidanceParameters>()),
    planner_data(std::make_shared<PlannerData>()),
    operation_mode(std::make_shared<autoware_adapi_v1_msgs::msg::OperationModeState>()),
    self_odometry(std::make_shared<nav_msgs::msg::Odometry>())
  {
    parameters->max_shift_length = 0.1;
    parameters->min_forward_distance = 0.0;

    const auto route = loadLaneChangeRoute();
    planner_data->route_handler = makeRouteHandler(route);
    planner_data->prev_route_id = route.uuid;
    planner_data->parameters.vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      0.3, 0.2, 2.5, 1.4, 0.5, 0.5, 0.2, 0.2, 1.5, 0.5);
    planner_data->parameters.vehicle_width = planner_data->parameters.vehicle_info.vehicle_width_m;
    planner_data->parameters.base_link2front =
      planner_data->parameters.vehicle_info.max_longitudinal_offset_m;
    planner_data->parameters.base_link2rear =
      -planner_data->parameters.vehicle_info.min_longitudinal_offset_m;
    planner_data->parameters.wheel_base = planner_data->parameters.vehicle_info.wheel_base_m;
    planner_data->parameters.backward_path_length = 10.0;
    planner_data->parameters.forward_path_length = 100.0;
    planner_data->parameters.ego_nearest_dist_threshold = 3.0;
    planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
    operation_mode->mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
    operation_mode->is_autoware_control_enabled = true;
    planner_data->operation_mode = operation_mode;

    const auto start_pose = autoware::test_utils::createPose(-50.0, 1.75, 0.0, 0.0, 0.0, 0.0);
    lanelet::ConstLanelet closest_lane;
    EXPECT_TRUE(planner_data->route_handler->getClosestLaneletWithinRoute(
      start_pose, &closest_lane));
    const auto lanes = planner_data->route_handler->getLaneletSequence(
      closest_lane, start_pose, planner_data->parameters.backward_path_length,
      planner_data->parameters.forward_path_length);
    EXPECT_FALSE(lanes.empty());
    const auto adjacent_lane = planner_data->route_handler->getRightLanelet(closest_lane, false, false);
    EXPECT_TRUE(adjacent_lane.has_value());
    const auto adjacent_lanes = planner_data->route_handler->getLaneletSequence(
      *adjacent_lane, start_pose, planner_data->parameters.backward_path_length,
      planner_data->parameters.forward_path_length);
    EXPECT_FALSE(adjacent_lanes.empty());

    upstream.path = planner_data->route_handler->getCenterLinePath(
      lanes, 0.0, std::numeric_limits<double>::max());
    EXPECT_GT(upstream.path.points.size(), 25U);
    adjacent_path = planner_data->route_handler->getCenterLinePath(
      adjacent_lanes, 0.0, std::numeric_limits<double>::max());
    EXPECT_GT(adjacent_path.points.size(), 20U);
    // Keep the recovery horizon inside this short test route while retaining enough points for
    // the target, the manual bypass, and the return maneuver.
    upstream.path.points.resize(35);
    for (auto & point : upstream.path.points) {
      point.point.longitudinal_velocity_mps = 1.0;
    }
    upstream.reference_path = upstream.path;

    self_odometry->pose.pose = upstream.path.points.front().point.pose;
    planner_data->self_odometry = self_odometry;
    planner_data->dynamic_object = makeObstacleAt(upstream.path.points.at(10));

    planning_factor_interface =
      std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
        &node, "simple_lane_change_avoidance_reengagement_test");
    module = std::make_unique<SimpleLaneChangeAvoidanceModule>(
      "simple_lane_change_avoidance", node, parameters, nullptr, rtc_interfaces,
      marker_interfaces, planning_factor_interface);
    module->setData(planner_data);
    module->setPreviousModuleOutput(upstream);
    module->onEntry();
  }

  void moveManualToSource(const size_t index)
  {
    operation_mode->is_autoware_control_enabled = false;
    self_odometry->pose.pose = upstream.path.points.at(index).point.pose;
    module->updateData();
  }

  void moveManualToAdjacent(const size_t index, const double offset)
  {
    operation_mode->is_autoware_control_enabled = false;
    moveToAdjacent(index, offset);
  }

  void moveToAdjacent(const size_t index, const double offset)
  {
    const auto source_pose = upstream.path.points.at(index).point.pose;
    const auto adjacent_pose = adjacent_path.points.at(index).point.pose;
    const double center_distance = std::hypot(
      adjacent_pose.position.x - source_pose.position.x,
      adjacent_pose.position.y - source_pose.position.y);
    auto manual_pose = source_pose;
    const double ratio = offset / center_distance;
    manual_pose.position.x += ratio * (adjacent_pose.position.x - source_pose.position.x);
    manual_pose.position.y += ratio * (adjacent_pose.position.y - source_pose.position.y);
    self_odometry->pose.pose = manual_pose;
    module->updateData();
  }

  void moveManualOutsideAdjacent(const size_t index)
  {
    operation_mode->is_autoware_control_enabled = false;
    const auto source_pose = upstream.path.points.at(index).point.pose;
    const auto adjacent_pose = adjacent_path.points.at(index).point.pose;
    const double center_distance = std::hypot(
      adjacent_pose.position.x - source_pose.position.x,
      adjacent_pose.position.y - source_pose.position.y);
    auto outside_pose = adjacent_pose;
    outside_pose.position.x +=
      2.0 * (adjacent_pose.position.x - source_pose.position.x) / center_distance;
    outside_pose.position.y +=
      2.0 * (adjacent_pose.position.y - source_pose.position.y) / center_distance;
    self_odometry->pose.pose = outside_pose;
    module->updateData();
  }

  void moveAutonomousToSource(const size_t index)
  {
    self_odometry->pose.pose = upstream.path.points.at(index).point.pose;
  }

  BehaviorModuleOutput reengage()
  {
    operation_mode->is_autoware_control_enabled = true;
    return module->run();
  }

  std::string latestPlanningFactorDetail() const
  {
    const auto factors = planning_factor_interface->get_factors();
    return factors.empty() ? std::string{} : factors.back().detail;
  }

  rclcpp::Node node;
  std::shared_ptr<SimpleLCAvoidanceParameters> parameters;
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  std::shared_ptr<PlannerData> planner_data;
  std::shared_ptr<autoware_adapi_v1_msgs::msg::OperationModeState> operation_mode;
  std::shared_ptr<nav_msgs::msg::Odometry> self_odometry;
  BehaviorModuleOutput upstream;
  PathWithLaneId adjacent_path;
  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface;
  std::unique_ptr<SimpleLaneChangeAvoidanceModule> module;
};

}  // namespace

class SimpleLaneChangeAvoidanceSceneTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(
  SimpleLaneChangeAvoidanceSceneTest, RunFallsBackToCachedReferenceWhenCurrentUpstreamPathIsEmpty)
{
  rclcpp::Node node{"simple_lane_change_avoidance_scene_test"};
  auto parameters = std::make_shared<SimpleLCAvoidanceParameters>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleLaneChangeAvoidanceModule module{
    "simple_lane_change_avoidance", node, parameters, nullptr, rtc_interfaces, marker_interfaces,
    nullptr};

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  planner_data->self_odometry = odometry;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  module.setPreviousModuleOutput(makeStraightOutput());
  const auto initial_output = module.run();
  ASSERT_FALSE(initial_output.path.points.empty());

  module.setPreviousModuleOutput(BehaviorModuleOutput{});
  BehaviorModuleOutput fallback_output;
  ASSERT_NO_THROW(fallback_output = module.run());
  EXPECT_FALSE(fallback_output.path.points.empty());
}

TEST_F(SimpleLaneChangeAvoidanceSceneTest, ObstacleWithoutAdjacentLaneProducesSafeStop)
{
  rclcpp::Node node{"simple_lane_change_avoidance_no_adjacent_lane_test"};
  auto parameters = std::make_shared<SimpleLCAvoidanceParameters>();
  parameters->min_forward_distance = 0.0;
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  auto planning_factor_interface =
    std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "simple_lane_change_avoidance_no_adjacent_lane_test");
  SimpleLaneChangeAvoidanceModule module{
    "simple_lane_change_avoidance", node, parameters, nullptr, rtc_interfaces, marker_interfaces,
    planning_factor_interface};

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeStaticObstacle();
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);
  module.setPreviousModuleOutput(makeStraightOutput());

  // The manager asks for execution before the scene has received its first updateData() call.
  // Detection must therefore use the upstream path rather than an as-yet empty cached reference.
  EXPECT_TRUE(module.isExecutionRequested());

  BehaviorModuleOutput output;
  ASSERT_NO_THROW(output = module.run());
  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(std::any_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
    return p.point.longitudinal_velocity_mps == 0.0;
  }));
  ASSERT_FALSE(planning_factor_interface->get_factors().empty());
  EXPECT_NE(
    planning_factor_interface->get_factors().back().detail.find("no_adjacent_lane"),
    std::string::npos);
}

TEST_F(SimpleLaneChangeAvoidanceSceneTest, EmptyInputsProduceNonEmptySafeStop)
{
  rclcpp::Node node{"simple_lane_change_avoidance_empty_input_test"};
  auto parameters = std::make_shared<SimpleLCAvoidanceParameters>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleLaneChangeAvoidanceModule module{
    "simple_lane_change_avoidance", node, parameters, nullptr, rtc_interfaces, marker_interfaces,
    nullptr};

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  planner_data->self_odometry = odometry;
  module.setData(planner_data);
  module.setPreviousModuleOutput(BehaviorModuleOutput{});

  const auto output = module.run();

  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(std::all_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
    return p.point.longitudinal_velocity_mps == 0.0;
  }));
}

TEST_F(
  SimpleLaneChangeAvoidanceSceneTest,
  ReengageAfterManualPassingOfNoRoomTargetGeneratesRecoveryPath)
{
  NoRoomReengagementScenario scenario{"simple_lane_change_avoidance_reengage_test"};
  const auto stopped_output = scenario.module->run();
  ASSERT_FALSE(stopped_output.path.points.empty());
  EXPECT_DOUBLE_EQ(stopped_output.path.points.back().point.longitudinal_velocity_mps, 0.0);
  scenario.module->updateCurrentState();
  // Keep the target within the tracker's spatial association window while placing the ego just
  // beyond it. This verifies that re-association does not clear the passed state.
  scenario.moveManualToAdjacent(13, 0.79);
  const auto recovered_output = scenario.reengage();
  ASSERT_FALSE(recovered_output.path.points.empty());
  EXPECT_GT(recovered_output.path.points.back().point.longitudinal_velocity_mps, 0.1);

  scenario.module->updateCurrentState();
  scenario.moveAutonomousToSource(28);
  for (size_t i = 0; i < 3; ++i) {
    const auto centered_output = scenario.module->run();
    ASSERT_FALSE(centered_output.path.points.empty());
    EXPECT_GT(centered_output.path.points.back().point.longitudinal_velocity_mps, 0.1);
    scenario.module->updateCurrentState();
  }
  EXPECT_EQ(scenario.module->getCurrentStatus(), ModuleStatus::SUCCESS);
}

TEST_F(
  SimpleLaneChangeAvoidanceSceneTest,
  PassingTargetReconcilesStateWithoutOperationModeTransition)
{
  NoRoomReengagementScenario scenario{"simple_lane_change_avoidance_periodic_state_test"};
  scenario.parameters->max_shift_length = 4.5;
  scenario.parameters->min_prepare_distance = 1.0;
  scenario.parameters->min_shifting_distance = 2.0;
  scenario.parameters->shifting_lateral_jerk = 2.0;
  scenario.parameters->return_distance_after_object = 2.0;
  scenario.parameters->road_boundary_margin = 0.0;
  scenario.planner_data->parameters.vehicle_info =
    autoware::vehicle_info_utils::createVehicleInfo(
      0.1, 0.1, 1.0, 0.2, 0.1, 0.1, 0.1, 0.1, 0.5, 0.2);
  scenario.planner_data->parameters.vehicle_width =
    scenario.planner_data->parameters.vehicle_info.vehicle_width_m;
  scenario.self_odometry->pose.pose = scenario.upstream.path.points.at(3).point.pose;
  scenario.planner_data->dynamic_object = makeObstacleAt(scenario.upstream.path.points.at(21));

  const auto avoidance_output = scenario.module->run();
  ASSERT_FALSE(avoidance_output.path.points.empty());
  ASSERT_GT(avoidance_output.path.points.back().point.longitudinal_velocity_mps, 0.1);
  scenario.module->updateCurrentState();

  scenario.moveToAdjacent(26, 1.43);
  const auto recovery_output = scenario.module->run();

  ASSERT_FALSE(recovery_output.path.points.empty());
  EXPECT_GT(recovery_output.path.points.back().point.longitudinal_velocity_mps, 0.1);
  const auto recovery_ego_index = autoware::motion_utils::findNearestIndex(
    recovery_output.path.points, scenario.self_odometry->pose.pose.position);
  EXPECT_LT(
    autoware_utils::calc_distance2d(
      recovery_output.path.points.at(recovery_ego_index).point.pose,
      scenario.self_odometry->pose.pose),
    0.6);
  EXPECT_LT(
    std::abs(autoware::motion_utils::calcLateralOffset(
      recovery_output.path.points, scenario.self_odometry->pose.pose.position)),
    0.1);
  EXPECT_EQ(
    scenario.operation_mode->mode,
    autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);
  EXPECT_TRUE(scenario.operation_mode->is_autoware_control_enabled);
}

TEST_F(
  SimpleLaneChangeAvoidanceSceneTest,
  LostTargetIsMarkedPassedFromHeldPoseWithoutOperationModeTransition)
{
  NoRoomReengagementScenario scenario{"simple_lane_change_avoidance_lost_passed_target_test"};
  const auto stopped_output = scenario.module->run();
  ASSERT_FALSE(stopped_output.path.points.empty());
  EXPECT_DOUBLE_EQ(stopped_output.path.points.back().point.longitudinal_velocity_mps, 0.0);
  scenario.module->updateCurrentState();

  scenario.planner_data->dynamic_object =
    std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  scenario.moveToAdjacent(20, 0.79);
  const auto recovery_output = scenario.module->run();

  ASSERT_FALSE(recovery_output.path.points.empty());
  EXPECT_GT(recovery_output.path.points.back().point.longitudinal_velocity_mps, 0.1);
  EXPECT_TRUE(scenario.operation_mode->is_autoware_control_enabled);
}

TEST_F(SimpleLaneChangeAvoidanceSceneTest, ReengageBeforeTargetKeepsSafetyStop)
{
  NoRoomReengagementScenario scenario{"simple_lane_change_avoidance_reengage_before_target_test"};
  ASSERT_FALSE(scenario.module->run().path.points.empty());
  scenario.module->updateCurrentState();

  // Keep the ego just before the target so the stop-before-target contract is unambiguous even
  // after the stop point's longitudinal distance is recomputed from the new ego pose.
  scenario.moveManualToSource(9);
  const auto output = scenario.reengage();
  ASSERT_FALSE(output.path.points.empty());
  EXPECT_DOUBLE_EQ(output.path.points.back().point.longitudinal_velocity_mps, 0.0);
  EXPECT_NE(scenario.latestPlanningFactorDetail().find("no_room"), std::string::npos);
}

TEST_F(SimpleLaneChangeAvoidanceSceneTest, PassedTargetWhileCenteredCompletesAfterStableCycles)
{
  NoRoomReengagementScenario scenario{"simple_lane_change_avoidance_reengage_centered_test"};
  ASSERT_FALSE(scenario.module->run().path.points.empty());
  scenario.module->updateCurrentState();

  scenario.moveManualToSource(20);
  for (size_t i = 0; i < 3; ++i) {
    const auto output = scenario.reengage();
    ASSERT_FALSE(output.path.points.empty());
    EXPECT_GT(output.path.points.back().point.longitudinal_velocity_mps, 0.1);
    scenario.module->updateCurrentState();
  }
  EXPECT_EQ(scenario.module->getCurrentStatus(), ModuleStatus::SUCCESS);
}

TEST_F(SimpleLaneChangeAvoidanceSceneTest, ReengageWithFootprintOutsideBothLanesKeepsBoundaryStop)
{
  NoRoomReengagementScenario scenario{"simple_lane_change_avoidance_reengage_boundary_test"};
  ASSERT_FALSE(scenario.module->run().path.points.empty());
  scenario.module->updateCurrentState();

  scenario.moveManualOutsideAdjacent(20);
  const auto output = scenario.reengage();
  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(std::all_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
    return p.point.longitudinal_velocity_mps == 0.0;
  }));
  EXPECT_NE(
    scenario.latestPlanningFactorDetail().find("footprint_out_of_boundary"), std::string::npos);
}

}  // namespace autoware::behavior_path_planner
