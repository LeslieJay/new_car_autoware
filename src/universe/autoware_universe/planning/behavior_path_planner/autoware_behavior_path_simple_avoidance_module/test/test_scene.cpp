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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
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
  output.reference_path = output.path;
  return output;
}

void addStraightLaneBounds(BehaviorModuleOutput & output, const double half_width)
{
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
  parameters->min_prepare_distance = 5.0;
  parameters->min_shifting_distance = 10.0;
  parameters->shifting_lateral_jerk = 0.5;
  parameters->min_shifting_speed = 1.0;
  parameters->return_distance_after_object = 5.0;
  return parameters;
}

autoware_perception_msgs::msg::PredictedObjects::SharedPtr makeStaticObstacle(
  const double x = 25.0, const uint8_t id = 0)
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
  objects->objects.push_back(object);
  return objects;
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

TEST_F(SimpleAvoidanceSceneTest, CandidateGenerationFailureUsesCachedReferencePath)
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
  failing_parameters->min_prepare_distance = 0.0;
  failing_parameters->min_shifting_distance = 0.0;
  failing_parameters->shifting_lateral_jerk = 1.0e9;
  failing_parameters->min_shifting_speed = 0.01;
  module.updateModuleParams(failing_parameters);
  module.setPreviousModuleOutput(BehaviorModuleOutput{});

  BehaviorModuleOutput fallback_output;
  ASSERT_NO_THROW(fallback_output = module.run());
  ASSERT_EQ(fallback_output.path.points.size(), upstream_output.path.points.size());
  EXPECT_EQ(fallback_output.path.points.front(), upstream_output.path.points.front());
  EXPECT_EQ(fallback_output.path.points.back(), upstream_output.path.points.back());
}

TEST_F(SimpleAvoidanceSceneTest, CommittedAvoidanceContinuesAfterTargetLossAndPathRollover)
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

  // The same target is now too close to create a fresh avoidance maneuver. Once execution is
  // committed, that infeasibility must not discard the already active shifted path.
  odometry->pose.pose.position.x = 16.0;
  module.setPreviousModuleOutput(initial_upstream);
  const auto close_target_output = module.run();
  ASSERT_TRUE(is_driving_path(close_target_output));
  EXPECT_LT(lateral_offset_near(close_target_output, 16.0), -0.1);

  planner_data->dynamic_object =
    std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  module.setPreviousModuleOutput(initial_upstream);
  const auto lost_target_output = module.run();
  ASSERT_TRUE(is_driving_path(lost_target_output));
  EXPECT_LT(lateral_offset_near(lost_target_output, 16.0), -0.1);
  EXPECT_LT(
    std::min_element(
      lost_target_output.path.points.begin(), lost_target_output.path.points.end(),
      [](const auto & left, const auto & right) {
        return left.point.pose.position.y < right.point.pose.position.y;
      })
      ->point.pose.position.y,
    -0.8);

  odometry->pose.pose.position.x = 42.0;
  // Reproduce the 2026-09-01 corner case: the generated return shift has reached zero, but the
  // physical vehicle is still laterally displaced. Publishing the upstream centerline here makes
  // planning_validator observe a > 0.5 m sudden trajectory shift and latch its soft stop.
  odometry->pose.pose.position.y = 1.2;
  module.setPreviousModuleOutput(initial_upstream);
  const auto lagging_return_output = module.run();
  ASSERT_TRUE(is_driving_path(lagging_return_output));
  EXPECT_LT(
    std::abs(lateral_offset_near(lagging_return_output, 42.0) - odometry->pose.pose.position.y),
    0.5);

  odometry->pose.pose.position.y = 0.0;
  module.setPreviousModuleOutput(initial_upstream);
  const auto completed_return_output = module.run();
  ASSERT_TRUE(is_driving_path(completed_return_output));
  EXPECT_NEAR(lateral_offset_near(completed_return_output, 42.0), 0.0, 0.05);

  const auto rolled_upstream = makeStraightOutput(31, 55.0);
  odometry->pose.pose.position.x = 60.0;
  module.setPreviousModuleOutput(rolled_upstream);
  const auto rolled_output = module.run();

  ASSERT_TRUE(is_driving_path(rolled_output));
  EXPECT_NEAR(lateral_offset_near(rolled_output, 60.0), 0.0, 0.05);
  EXPECT_GE(rolled_output.path.points.back().point.pose.position.x, 80.0);

  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), ModuleStatus::RUNNING);
  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), ModuleStatus::RUNNING);
  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), ModuleStatus::SUCCESS);
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
  planner_data->dynamic_object = makeStaticObstacle(41.0, 2);
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

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
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
  module.setData(planner_data);

  auto upstream_output = makeStraightOutput(61);
  addStraightLaneBounds(upstream_output, 0.75);
  module.setPreviousModuleOutput(upstream_output);
  module.onEntry();

  const auto output = module.run();

  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(std::all_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
    return p.point.longitudinal_velocity_mps == 0.0;
  })) << "a candidate that puts the vehicle footprint outside the lane must be rejected";
}

TEST_F(SimpleAvoidanceSceneTest, CommittedPathCrossingUpdatedLaneBoundaryProducesSafeStop)
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

  auto planner_data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = 1.0;
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
  EXPECT_TRUE(std::all_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
    return p.point.longitudinal_velocity_mps == 0.0;
  })) << "an already committed path must be revalidated against updated lane boundaries";
  EXPECT_TRUE(std::all_of(output.path.points.begin(), output.path.points.end(), [](const auto & p) {
    return std::abs(p.point.pose.position.y) < 1.0e-6;
  })) << "the safe-stop path must use in-lane geometry, not the rejected shifted path";
}

}  // namespace autoware::behavior_path_planner
