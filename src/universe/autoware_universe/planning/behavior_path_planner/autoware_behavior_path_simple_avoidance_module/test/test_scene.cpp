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

autoware_perception_msgs::msg::PredictedObjects::SharedPtr makeStaticObstacle()
{
  auto objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 25.0;
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

  planner_data->dynamic_object =
    std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  odometry->pose.pose.position.x = 16.0;
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

}  // namespace autoware::behavior_path_planner
