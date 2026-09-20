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

#include "autoware/behavior_path_simple_avoidance_module/scene.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
namespace
{
BehaviorModuleOutput makeStraightOutput(const size_t point_count = 4, const double start_x = 0.0)
{
  BehaviorModuleOutput output;
  for (size_t i = 0; i < point_count; ++i) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = start_x + static_cast<double>(i);
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 1.0;
    output.path.points.push_back(point);
  }
  output.path.header.frame_id = "map";
  for (const auto & path_point : output.path.points) {
    auto left = path_point.point.pose.position;
    left.y = 2.0;
    output.path.left_bound.push_back(left);

    auto right = path_point.point.pose.position;
    right.y = -2.0;
    output.path.right_bound.push_back(right);
  }
  output.reference_path = output.path;
  return output;
}

void addStraightLaneBounds(BehaviorModuleOutput & output, const double half_width)
{
  output.path.left_bound.clear();
  output.path.right_bound.clear();
  for (const auto & path_point : output.path.points) {
    auto left = path_point.point.pose.position;
    left.y = half_width;
    output.path.left_bound.push_back(left);

    auto right = path_point.point.pose.position;
    right.y = -half_width;
    output.path.right_bound.push_back(right);
  }
  output.reference_path = output.path;
}

std::shared_ptr<SimpleAvoidanceParameters> makeParameters()
{
  auto parameters = std::make_shared<SimpleAvoidanceParameters>();
  parameters->lateral_margin = 0.4;
  parameters->max_shift_length = 4.0;
  parameters->avoidance_start_distance_before_object_front = 10.0;
  parameters->min_shifting_distance = 10.0;
  parameters->shifting_lateral_jerk = 0.5;
  parameters->min_shifting_speed = 1.0;
  parameters->return_distance_after_object = 5.0;
  return parameters;
}

autoware_perception_msgs::msg::PredictedObjects::SharedPtr makeStaticObstacle(
  const double x = 25.0, const uint8_t id = 0, const double speed = 0.0)
{
  auto objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  autoware_perception_msgs::msg::PredictedObject object;
  object.object_id.uuid.front() = id;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 0.5;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
  objects->objects.push_back(object);
  return objects;
}

autoware_planning_msgs::msg::LaneletRoute loadLaneChangeRoute()
{
  const auto route_path = autoware::test_utils::get_absolute_path_to_route(
    "autoware_route_handler", "lane_change_test_route.yaml");
  return autoware::test_utils::parse<autoware_planning_msgs::msg::LaneletRoute>(
    YAML::LoadFile(route_path));
}

std::shared_ptr<autoware::route_handler::RouteHandler> makeRouteHandler(
  const autoware_planning_msgs::msg::LaneletRoute & route)
{
  const auto map_path =
    autoware::test_utils::get_absolute_path_to_lanelet_map("autoware_test_utils", "2km_test.osm");
  const auto map_bin = autoware::test_utils::make_map_bin_msg(map_path, 1.0);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin);
  route_handler->setRoute(route);
  return route_handler;
}

autoware_perception_msgs::msg::PredictedObjects::SharedPtr makeObstacleAt(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & path_point,
  const uint8_t id = 0, const double speed = 0.0)
{
  auto objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  autoware_perception_msgs::msg::PredictedObject object;
  object.object_id.uuid.front() = id;
  object.kinematics.initial_pose_with_covariance.pose = path_point.point.pose;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  objects->objects.push_back(object);
  return objects;
}

std::shared_ptr<PlannerData> makeBoundaryTestPlannerData(
  const std::shared_ptr<nav_msgs::msg::Odometry> & odometry)
{
  auto planner_data = std::make_shared<PlannerData>();
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeStaticObstacle();
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.vehicle_info.wheel_tread_m = 0.6;
  planner_data->parameters.vehicle_info.left_overhang_m = 0.2;
  planner_data->parameters.vehicle_info.right_overhang_m = 0.2;
  planner_data->parameters.vehicle_info.wheel_base_m = 1.0;
  planner_data->parameters.vehicle_info.front_overhang_m = 0.5;
  planner_data->parameters.vehicle_info.rear_overhang_m = 0.5;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  return planner_data;
}
}  // namespace

class SimpleAvoidanceSceneTest : public ::testing::Test
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

TEST_F(SimpleAvoidanceSceneTest, CandidateGenerationFailurePublishesSafeStop)
{
  rclcpp::Node node{"simple_avoidance_candidate_failure_test"};
  auto parameters = makeParameters();
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

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

  const auto upstream_output = makeStraightOutput(61);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();
  ASSERT_FALSE(module.run().path.points.empty());

  auto failing_parameters = std::make_shared<SimpleAvoidanceParameters>(*parameters);
  failing_parameters->avoidance_start_distance_before_object_front = 0.0;
  failing_parameters->min_shifting_distance = 0.0;
  failing_parameters->shifting_lateral_jerk = 1.0e9;
  failing_parameters->min_shifting_speed = 0.01;
  module.updateModuleParams(failing_parameters);
  module.setPreviousModuleOutput(BehaviorModuleOutput{});

  BehaviorModuleOutput fallback_output;
  ASSERT_NO_THROW(fallback_output = module.run());
  ASSERT_FALSE(fallback_output.path.points.empty());
  EXPECT_TRUE(
    std::all_of(
      fallback_output.path.points.begin(), fallback_output.path.points.end(),
      [](const auto & point) { return point.point.longitudinal_velocity_mps == 0.0; }));
}

TEST_F(SimpleAvoidanceSceneTest, InfeasibleNewTargetStopsOnlyWhenLongitudinalStopIsFeasible)
{
  rclcpp::Node node{"simple_avoidance_conditional_stop_test"};
  auto parameters = makeParameters();
  parameters->stop_max_jerk = 1.0;
  parameters->stop_low_speed_threshold = 0.2;
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  planner_data->self_odometry = odometry;
  auto acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  planner_data->self_acceleration = acceleration;
  planner_data->parameters.min_acc = -1.0;
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.base_link2front = 0.5;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  const auto upstream_output = makeStraightOutput(61);
  planner_data->dynamic_object = makeStaticObstacle(8.0, 41);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();
  const auto stopped_output = module.run();
  ASSERT_FALSE(stopped_output.path.points.empty());
  EXPECT_TRUE(
    std::any_of(
      stopped_output.path.points.begin(), stopped_output.path.points.end(),
      [](const auto & point) { return point.point.longitudinal_velocity_mps == 0.0; }));

  module.onExit();
  planner_data->dynamic_object = makeStaticObstacle(0.75, 42);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();
  const auto pass_through_output = module.run();
  ASSERT_EQ(pass_through_output.path.points.size(), upstream_output.path.points.size());
  EXPECT_TRUE(
    std::all_of(
      pass_through_output.path.points.begin(), pass_through_output.path.points.end(),
      [](const auto & point) { return point.point.longitudinal_velocity_mps > 0.0; }));

  module.onExit();
  odometry->twist.twist.linear.x = 2.0;
  planner_data->self_acceleration.reset();
  planner_data->dynamic_object = makeStaticObstacle(8.0, 43);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();
  const auto missing_acceleration_output = module.run();
  EXPECT_TRUE(
    std::all_of(
      missing_acceleration_output.path.points.begin(),
      missing_acceleration_output.path.points.end(),
      [](const auto & point) { return point.point.longitudinal_velocity_mps > 0.0; }));
}

TEST_F(SimpleAvoidanceSceneTest, CandidateCanUseShiftSideAdjacentLaneAndPublishesItAsDrivable)
{
  rclcpp::Node node{"simple_avoidance_adjacent_lane_test"};
  auto parameters = makeParameters();
  parameters->min_shifting_distance = 5.0;
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  const auto route = loadLaneChangeRoute();
  auto planner_data = std::make_shared<PlannerData>();
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
  planner_data->parameters.front_overhang = planner_data->parameters.vehicle_info.front_overhang_m;
  planner_data->parameters.rear_overhang = planner_data->parameters.vehicle_info.rear_overhang_m;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;

  const auto start_pose = autoware::test_utils::createPose(-50.0, 1.75, 0.0, 0.0, 0.0, 0.0);
  lanelet::ConstLanelet source_lane;
  ASSERT_TRUE(planner_data->route_handler->getClosestLaneletWithinRoute(start_pose, &source_lane));
  const auto source_lanes = planner_data->route_handler->getLaneletSequence(
    source_lane, start_pose, planner_data->parameters.backward_path_length,
    planner_data->parameters.forward_path_length);
  ASSERT_FALSE(source_lanes.empty());
  const auto right_lane = planner_data->route_handler->getRightLanelet(source_lane, false, false);
  ASSERT_TRUE(right_lane.has_value());

  BehaviorModuleOutput upstream;
  upstream.path = planner_data->route_handler->getCenterLinePath(
    source_lanes, 0.0, std::numeric_limits<double>::max());
  ASSERT_GT(upstream.path.points.size(), 30U);
  upstream.path.points.resize(35);
  for (auto & point : upstream.path.points) {
    point.point.longitudinal_velocity_mps = 1.0;
  }
  upstream.reference_path = upstream.path;

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose = upstream.path.points.front().point.pose;
  odometry->twist.twist.linear.x = 1.0;
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeObstacleAt(upstream.path.points.at(20), 51);
  module.setData(planner_data);
  module.setPreviousModuleOutput(upstream);
  module.onEntry();

  const auto output = module.run();
  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(
    std::all_of(output.path.points.begin(), output.path.points.end(), [](const auto & point) {
      return point.point.longitudinal_velocity_mps > 0.0;
    }));
  std::string drivable_lane_ids;
  for (const auto & lanes : output.drivable_area_info.drivable_lanes) {
    drivable_lane_ids +=
      std::to_string(lanes.left_lane.id()) + "/" + std::to_string(lanes.right_lane.id()) + " ";
  }
  EXPECT_TRUE(
    std::any_of(
      output.drivable_area_info.drivable_lanes.begin(),
      output.drivable_area_info.drivable_lanes.end(),
      [&](const auto & lanes) {
        if (lanes.left_lane.id() == lanes.right_lane.id()) {
          return false;
        }
        const auto expected_right =
          planner_data->route_handler->getRightLanelet(lanes.left_lane, false, false);
        return expected_right.has_value() && expected_right->id() == lanes.right_lane.id();
      }))
    << "initial adjacent=" << right_lane->id() << " output=" << drivable_lane_ids;

  const auto adjacent_lanes = planner_data->route_handler->getLaneletSequence(
    *right_lane, start_pose, planner_data->parameters.backward_path_length,
    planner_data->parameters.forward_path_length);
  const auto adjacent_path = planner_data->route_handler->getCenterLinePath(
    adjacent_lanes, 0.0, std::numeric_limits<double>::max());
  ASSERT_GT(adjacent_path.points.size(), 15U);
  for (const double adjacent_object_speed : {0.0, 2.0}) {
    auto occupied_objects = makeObstacleAt(upstream.path.points.at(20), 51);
    const auto adjacent_object = makeObstacleAt(
      adjacent_path.points.at(10), adjacent_object_speed == 0.0 ? 52 : 53, adjacent_object_speed);
    occupied_objects->objects.push_back(adjacent_object->objects.front());
    planner_data->dynamic_object = occupied_objects;
    module.onExit();
    module.setPreviousModuleOutput(upstream);
    module.onEntry();
    const auto occupied_output = module.run();
    ASSERT_FALSE(occupied_output.path.points.empty());
    EXPECT_TRUE(
      std::any_of(
        occupied_output.path.points.begin(), occupied_output.path.points.end(),
        [](const auto & point) { return point.point.longitudinal_velocity_mps == 0.0; }))
      << "adjacent lane object speed=" << adjacent_object_speed;
  }

  auto objects_with_distant_rear_object = makeObstacleAt(upstream.path.points.at(20), 51);
  auto rear_pose = adjacent_path.points.front();
  rear_pose.point.pose.position.x -= planner_data->parameters.backward_path_length + 5.0;
  const auto distant_rear_object = makeObstacleAt(rear_pose, 54, 2.0);
  objects_with_distant_rear_object->objects.push_back(distant_rear_object->objects.front());
  planner_data->dynamic_object = objects_with_distant_rear_object;
  module.onExit();
  module.setPreviousModuleOutput(upstream);
  module.onEntry();
  const auto distant_rear_output = module.run();
  EXPECT_TRUE(
    std::all_of(
      distant_rear_output.path.points.begin(), distant_rear_output.path.points.end(),
      [](const auto & point) { return point.point.longitudinal_velocity_mps > 0.0; }));
}

TEST_F(SimpleAvoidanceSceneTest, CommittedAvoidancePassesThroughAfterTargetLoss)
{
  rclcpp::Node node{"simple_avoidance_committed_target_loss_test"};
  auto parameters = makeParameters();
  parameters->target_lost_time_threshold = 0.0;
  parameters->lateral_execution_threshold = 0.1;
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeStaticObstacle();
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  const auto initial_upstream = makeStraightOutput(61);
  module.setPreviousModuleOutput(initial_upstream);
  module.onEntry();
  module.updateCurrentState();
  ASSERT_EQ(module.getCurrentStatus(), ModuleStatus::RUNNING);
  ASSERT_FALSE(module.run().path.points.empty());

  const auto lateral_offset_near = [](const BehaviorModuleOutput & output, const double x) {
    const auto closest = std::min_element(
      output.path.points.begin(), output.path.points.end(),
      [x](const auto & left, const auto & right) {
        return std::abs(left.point.pose.position.x - x) < std::abs(right.point.pose.position.x - x);
      });
    return closest->point.pose.position.y;
  };
  const auto is_driving_path = [](const BehaviorModuleOutput & output) {
    return !output.path.points.empty() &&
           std::all_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
             return p.point.longitudinal_velocity_mps > 0.0;
           });
  };
  planner_data->dynamic_object =
    std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  module.setPreviousModuleOutput(initial_upstream);
  const auto cancelled_candidate_output = module.run();
  ASSERT_TRUE(is_driving_path(cancelled_candidate_output));
  EXPECT_NEAR(lateral_offset_near(cancelled_candidate_output, 16.0), 0.0, 1.0e-6);

  planner_data->dynamic_object = makeStaticObstacle();
  module.setPreviousModuleOutput(initial_upstream);
  ASSERT_FALSE(module.run().path.points.empty());

  odometry->pose.pose.position.x = 6.0;
  module.setPreviousModuleOutput(initial_upstream);
  ASSERT_FALSE(module.run().path.points.empty());

  // The same target is now too close to create a fresh avoidance maneuver. Once committed, the
  // module must keep the continuous avoidance path instead of inserting a new stop.
  odometry->pose.pose.position.x = 16.0;
  odometry->pose.pose.position.y = 0.5;
  module.setPreviousModuleOutput(initial_upstream);
  const auto close_target_output = module.run();
  ASSERT_TRUE(is_driving_path(close_target_output));

  planner_data->dynamic_object =
    std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  module.setPreviousModuleOutput(initial_upstream);
  const auto lost_target_output = module.run();
  ASSERT_TRUE(is_driving_path(lost_target_output));
  ASSERT_EQ(lost_target_output.path.points.size(), initial_upstream.path.points.size());
  for (size_t i = 0; i < initial_upstream.path.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(
      lost_target_output.path.points.at(i).point.pose.position.x,
      initial_upstream.path.points.at(i).point.pose.position.x);
    EXPECT_DOUBLE_EQ(
      lost_target_output.path.points.at(i).point.pose.position.y,
      initial_upstream.path.points.at(i).point.pose.position.y);
    EXPECT_DOUBLE_EQ(
      lost_target_output.path.points.at(i).point.longitudinal_velocity_mps,
      initial_upstream.path.points.at(i).point.longitudinal_velocity_mps);
  }

  // The old implementation stayed in RETURNING and waited for the measured lateral offset to
  // converge. The no-target fast path must finish the module in this same cycle instead.
  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), ModuleStatus::SUCCESS);
}

TEST_F(SimpleAvoidanceSceneTest, PassedTargetPassesThroughImmediately)
{
  rclcpp::Node node{"simple_avoidance_passed_target_test"};
  auto parameters = makeParameters();
  parameters->target_lost_time_threshold = 1.0;
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeStaticObstacle(25.0, 61);
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  const auto upstream = makeStraightOutput(61);
  module.setPreviousModuleOutput(upstream);
  module.onEntry();
  ASSERT_FALSE(module.run().path.points.empty());
  module.updateCurrentState();
  ASSERT_EQ(module.getCurrentStatus(), ModuleStatus::RUNNING);

  odometry->pose.pose.position.x = 6.0;
  module.setPreviousModuleOutput(upstream);
  ASSERT_FALSE(module.run().path.points.empty());

  // Commit the avoidance maneuver before moving past the target.
  odometry->pose.pose.position.x = 16.0;
  odometry->pose.pose.position.y = 0.5;
  module.setPreviousModuleOutput(upstream);
  ASSERT_FALSE(module.run().path.points.empty());

  // Keep the obstacle in the perception message, but place it behind the ego so the active
  // target is classified as passed rather than lost.
  odometry->pose.pose.position.x = 35.0;
  module.setPreviousModuleOutput(upstream);
  const auto passed_output = module.run();
  ASSERT_EQ(passed_output.path.points.size(), upstream.path.points.size());
  for (size_t i = 0; i < upstream.path.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(
      passed_output.path.points.at(i).point.pose.position.x,
      upstream.path.points.at(i).point.pose.position.x);
    EXPECT_DOUBLE_EQ(
      passed_output.path.points.at(i).point.pose.position.y,
      upstream.path.points.at(i).point.pose.position.y);
    EXPECT_DOUBLE_EQ(
      passed_output.path.points.at(i).point.longitudinal_velocity_mps,
      upstream.path.points.at(i).point.longitudinal_velocity_mps);
  }

  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), ModuleStatus::SUCCESS);
}

TEST_F(SimpleAvoidanceSceneTest, TargetLossPassThroughAllowsNewObstacle)
{
  rclcpp::Node node{"simple_avoidance_returning_new_target_test"};
  auto parameters = makeParameters();
  parameters->target_lost_time_threshold = 0.0;
  parameters->lateral_execution_threshold = 0.1;
  parameters->commitment_distance_before_shift_start = 2.0;
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  auto planner_data = std::make_shared<PlannerData>();
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeStaticObstacle(20.0, 31);
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  const auto upstream = makeStraightOutput(101);
  module.setPreviousModuleOutput(upstream);
  module.onEntry();
  ASSERT_FALSE(module.run().path.points.empty());
  module.updateCurrentState();
  ASSERT_EQ(module.getCurrentStatus(), ModuleStatus::RUNNING);

  // Enter the commitment window while the first target is still present.
  odometry->pose.pose.position.x = 8.0;
  module.setPreviousModuleOutput(upstream);
  ASSERT_FALSE(module.run().path.points.empty());

  // The first target is gone; the committed path is cleared and the upstream path is passed
  // through immediately.
  planner_data->dynamic_object =
    std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  odometry->pose.pose.position.x = 14.0;
  module.setPreviousModuleOutput(upstream);
  const auto returning_output = module.run();
  ASSERT_FALSE(returning_output.path.points.empty());
  EXPECT_TRUE(
    std::all_of(
      returning_output.path.points.begin(), returning_output.path.points.end(),
      [](const auto & point) { return std::abs(point.point.pose.position.y) < 1.0e-6; }));

  // A second target appearing before the vehicle returns to the center line is still eligible for
  // a fresh avoidance maneuver; clearing the first maneuver must not suppress target detection.
  planner_data->dynamic_object = makeStaticObstacle(50.0, 32);
  odometry->pose.pose.position.x = 18.0;
  module.setPreviousModuleOutput(upstream);
  const auto replanned_output = module.run();
  ASSERT_FALSE(replanned_output.path.points.empty());

  const auto lateral_offset_near = [](const BehaviorModuleOutput & output, const double x) {
    const auto closest = std::min_element(
      output.path.points.begin(), output.path.points.end(),
      [x](const auto & left, const auto & right) {
        return std::abs(left.point.pose.position.x - x) < std::abs(right.point.pose.position.x - x);
      });
    return closest->point.pose.position.y;
  };

  // The second target must produce a fresh shifted path rather than being ignored after the
  // previous no-target pass-through.
  EXPECT_LT(lateral_offset_near(replanned_output, 45.0), -0.1);
  EXPECT_LT(std::abs(lateral_offset_near(replanned_output, 45.0)), 2.0);
  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), ModuleStatus::RUNNING);
}

TEST_F(SimpleAvoidanceSceneTest, AvoidanceStartCommitmentWindowPreventsLateStop)
{
  rclcpp::Node node{"simple_avoidance_commitment_window_test"};
  auto parameters = makeParameters();
  parameters->lateral_margin = 0.5;
  parameters->max_shift_length = 3.0;
  parameters->avoidance_start_distance_before_object_front = 10.0;
  parameters->min_shifting_distance = 5.0;
  parameters->shifting_lateral_jerk = 0.8;
  parameters->min_shifting_speed = 1.2;
  parameters->lateral_execution_threshold = 0.2;
  parameters->commitment_distance_before_shift_start = 2.0;

  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  auto planner_data = std::make_shared<PlannerData>();
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeStaticObstacle(34.62, 35);
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  const auto upstream_output = makeStraightOutput(101);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();
  const auto initial_output = module.run();
  ASSERT_FALSE(initial_output.path.points.empty());

  const auto has_stop = [](const BehaviorModuleOutput & output) {
    return std::any_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
      return p.point.longitudinal_velocity_mps == 0.0;
    });
  };

  // This corresponds to the 35m runtime trace at target_lon ~= 11.62m and
  // dist_to_avoid_start ~= 1.12m. The candidate must be committed before the
  // discrete shift-line start index is reached.
  odometry->twist.twist.linear.x = 2.0;
  odometry->pose.pose.position.x = 23.0;
  module.setPreviousModuleOutput(upstream_output);
  const auto near_start_output = module.run();
  ASSERT_FALSE(near_start_output.path.points.empty());
  EXPECT_FALSE(has_stop(near_start_output));

  // At target_lon ~= 10.39m, the continuously computed start point is just
  // behind ego. A committed maneuver must keep the previously generated path
  // instead of converting it into a safety stop.
  odometry->pose.pose.position.x = 24.23;
  module.setPreviousModuleOutput(upstream_output);
  const auto boundary_output = module.run();
  ASSERT_FALSE(boundary_output.path.points.empty());
  EXPECT_FALSE(has_stop(boundary_output));
}

TEST_F(SimpleAvoidanceSceneTest, NewTargetReplanDoesNotReuseStaleBaseOffsetAtEgo)
{
  rclcpp::Node node{"simple_avoidance_stale_base_offset_test"};
  auto parameters = makeParameters();
  parameters->target_lost_time_threshold = 0.0;
  parameters->lateral_execution_threshold = 0.1;
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = makeStaticObstacle(40.0, 1);
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  const auto upstream_output = makeStraightOutput(81);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();
  ASSERT_FALSE(module.run().path.points.empty());

  // Move beyond the first avoidance shift. PathShifter folds its planned end shift into
  // base_offset, although odometry and the freshly supplied upstream path are still centered.
  odometry->pose.pose.position.x = 16.0;
  module.setPreviousModuleOutput(upstream_output);
  ASSERT_FALSE(module.run().path.points.empty());

  // A different, feasible target triggers a replan while the old planned base offset is stale.
  planner_data->dynamic_object = makeStaticObstacle(41.0, 2, 1.0);
  module.setPreviousModuleOutput(upstream_output);
  const auto replanned_output = module.run();
  ASSERT_FALSE(replanned_output.path.points.empty());

  const auto closest = std::min_element(
    replanned_output.path.points.begin(), replanned_output.path.points.end(),
    [&odometry](const auto & left, const auto & right) {
      const double ego_x = odometry->pose.pose.position.x;
      return std::abs(left.point.pose.position.x - ego_x) <
             std::abs(right.point.pose.position.x - ego_x);
    });
  ASSERT_NE(closest, replanned_output.path.points.end());
  EXPECT_LT(std::abs(closest->point.pose.position.y - odometry->pose.pose.position.y), 0.5)
    << "new-target replan must be anchored to measured ego pose, not the old planned base offset";

  const auto upstream_closest = std::min_element(
    upstream_output.path.points.begin(), upstream_output.path.points.end(),
    [&odometry](const auto & left, const auto & right) {
      const double ego_x = odometry->pose.pose.position.x;
      return std::abs(left.point.pose.position.x - ego_x) <
             std::abs(right.point.pose.position.x - ego_x);
    });
  ASSERT_NE(upstream_closest, upstream_output.path.points.end());
  EXPECT_LT(std::abs(closest->point.pose.position.y - upstream_closest->point.pose.position.y), 0.5)
    << "new-target replan must remain continuous with the last valid upstream path at ego";
}

TEST_F(SimpleAvoidanceSceneTest, AvoidanceStartTracksObjectFrontForDifferentDistances)
{
  const auto first_shift_x = [](const BehaviorModuleOutput & output) {
    const auto shifted = std::find_if(
      output.path.points.begin(), output.path.points.end(),
      [](const auto & point) { return std::abs(point.point.pose.position.y) > 1.0e-4; });
    return shifted == output.path.points.end() ? -1.0 : shifted->point.pose.position.x;
  };

  for (const double obstacle_x : {20.0, 40.0, 60.0}) {
    rclcpp::Node node{
      "simple_avoidance_front_relative_start_" + std::to_string(static_cast<int>(obstacle_x))};
    auto parameters = makeParameters();
    auto trailer_store = std::make_shared<TrailerConfigurationStore>();
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
      marker_interfaces;
    SimpleAvoidanceModule module{
      "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
      marker_interfaces,  nullptr};

    auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
    odometry->pose.pose.orientation.w = 1.0;
    odometry->twist.twist.linear.x = 1.0;
    auto planner_data = std::make_shared<PlannerData>();
    planner_data->self_odometry = odometry;
    planner_data->dynamic_object = makeStaticObstacle(obstacle_x, 10);
    planner_data->parameters.vehicle_width = 1.0;
    planner_data->parameters.backward_path_length = 10.0;
    planner_data->parameters.forward_path_length = 100.0;
    planner_data->parameters.input_path_interval = 1.0;
    planner_data->parameters.ego_nearest_dist_threshold = 3.0;
    planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
    module.setData(planner_data);

    module.setPreviousModuleOutput(makeStraightOutput(101));
    module.onEntry();
    const auto output = module.run();

    ASSERT_FALSE(output.path.points.empty());
    const double actual_start_x = first_shift_x(output);
    ASSERT_GT(actual_start_x, 0.0);
    // The obstacle front is obstacle_x - 0.5 and the configured minimum is
    // max(10.0, 10.0 + 0.4) = 10.4m. The path is sampled at approximately 1m.
    EXPECT_NEAR(actual_start_x, obstacle_x - 0.5 - 10.4, 2.0);
  }
}

TEST_F(SimpleAvoidanceSceneTest, InfeasibleNearestObstacleRemainsActiveTarget)
{
  rclcpp::Node node{"simple_avoidance_skips_infeasible_nearest_target"};
  auto parameters = makeParameters();
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  auto planner_data = std::make_shared<PlannerData>();
  planner_data->self_odometry = odometry;
  auto objects = makeStaticObstacle(10.0, 20);
  const auto farther_object = makeStaticObstacle(40.0, 21);
  objects->objects.push_back(farther_object->objects.front());
  planner_data->dynamic_object = objects;
  planner_data->parameters.vehicle_width = 1.0;
  planner_data->parameters.backward_path_length = 10.0;
  planner_data->parameters.forward_path_length = 100.0;
  planner_data->parameters.input_path_interval = 1.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  module.setData(planner_data);

  module.setPreviousModuleOutput(makeStraightOutput(81));
  module.onEntry();
  const auto output = module.run();

  ASSERT_FALSE(output.path.points.empty());
  const auto shifted = std::find_if(
    output.path.points.begin(), output.path.points.end(),
    [](const auto & point) { return std::abs(point.point.pose.position.y) > 0.01; });
  EXPECT_EQ(shifted, output.path.points.end())
    << "the farther feasible obstacle must not replace the nearest active target";
  EXPECT_TRUE(
    std::any_of(output.path.points.begin(), output.path.points.end(), [](const auto & point) {
      return point.point.longitudinal_velocity_mps == 0.0;
    }));
}

TEST_F(SimpleAvoidanceSceneTest, CandidateCrossingLaneBoundaryProducesSafeStop)
{
  rclcpp::Node node{"simple_avoidance_lane_boundary_test"};
  auto parameters = makeParameters();
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  auto planner_data = makeBoundaryTestPlannerData(odometry);
  planner_data->self_acceleration =
    std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  planner_data->parameters.min_acc = -1.0;
  module.setData(planner_data);

  auto upstream_output = makeStraightOutput(61);
  addStraightLaneBounds(upstream_output, 0.75);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();

  const auto output = module.run();

  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(
    std::any_of(
      output.path.points.begin(), output.path.points.end(),
      [](const auto & p) { return p.point.longitudinal_velocity_mps == 0.0; }))
    << "a boundary-related infeasible candidate must insert a stop before the target";
}

TEST_F(SimpleAvoidanceSceneTest, CandidateWithIncompleteBoundaryPassesThrough)
{
  rclcpp::Node node{"simple_avoidance_single_lane_boundary_test"};
  auto parameters = makeParameters();
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  auto planner_data = makeBoundaryTestPlannerData(odometry);
  module.setData(planner_data);

  auto upstream_output = makeStraightOutput(61);
  addStraightLaneBounds(upstream_output, 0.75);
  upstream_output.path.left_bound.clear();
  upstream_output.reference_path = upstream_output.path;
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();

  const auto output = module.run();

  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(
    std::all_of(
      output.path.points.begin(), output.path.points.end(),
      [](const auto & p) { return p.point.longitudinal_velocity_mps > 0.0; }))
    << "incomplete boundary data must not trigger an active stop";
}

TEST_F(SimpleAvoidanceSceneTest, CandidateWithoutAnyBoundaryPassesThrough)
{
  rclcpp::Node node{"simple_avoidance_missing_boundary_test"};
  auto parameters = makeParameters();
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  auto planner_data = makeBoundaryTestPlannerData(odometry);
  module.setData(planner_data);

  auto upstream_output = makeStraightOutput(61);
  upstream_output.path.left_bound.clear();
  upstream_output.path.right_bound.clear();
  upstream_output.reference_path = upstream_output.path;
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();

  const auto output = module.run();

  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(
    std::all_of(
      output.path.points.begin(), output.path.points.end(),
      [](const auto & p) { return p.point.longitudinal_velocity_mps > 0.0; }))
    << "missing boundary data must not trigger an active stop";
}

TEST_F(SimpleAvoidanceSceneTest, CommittedInvalidatedPathFallsBackWithoutStopping)
{
  rclcpp::Node node{"simple_avoidance_committed_lane_boundary_test"};
  auto parameters = makeParameters();
  parameters->target_lost_time_threshold = 0.0;
  auto trailer_store = std::make_shared<TrailerConfigurationStore>();
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleAvoidanceModule module{
    "simple_avoidance", node,   parameters, trailer_store, rtc_interfaces,
    marker_interfaces,  nullptr};

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
  auto planner_data = makeBoundaryTestPlannerData(odometry);
  module.setData(planner_data);

  auto wide_lane_output = makeStraightOutput(61);
  addStraightLaneBounds(wide_lane_output, 2.0);
  module.setPreviousModuleOutput(wide_lane_output);
  module.onEntry();
  ASSERT_FALSE(module.run().path.points.empty());

  odometry->pose.pose.position.x = 6.0;
  module.setPreviousModuleOutput(wide_lane_output);
  ASSERT_FALSE(module.run().path.points.empty());

  odometry->pose.pose.position.x = 16.0;
  module.setPreviousModuleOutput(wide_lane_output);
  ASSERT_FALSE(module.run().path.points.empty());

  planner_data->dynamic_object =
    std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  auto narrow_lane_output = makeStraightOutput(61);
  addStraightLaneBounds(narrow_lane_output, 0.75);
  module.setPreviousModuleOutput(narrow_lane_output);
  const auto output = module.run();

  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(
    std::all_of(
      output.path.points.begin(), output.path.points.end(),
      [](const auto & p) { return p.point.longitudinal_velocity_mps > 0.0; }))
    << "an already committed maneuver must not introduce a new stop";
  EXPECT_TRUE(
    std::all_of(
      output.path.points.begin(), output.path.points.end(),
      [](const auto & p) { return std::abs(p.point.pose.position.y) < 1.0e-6; }))
    << "the invalidated shifted path must not be reused";
}

}  // namespace autoware::behavior_path_planner
