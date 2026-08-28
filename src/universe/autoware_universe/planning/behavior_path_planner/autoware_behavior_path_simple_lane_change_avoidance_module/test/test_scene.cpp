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

#include <gtest/gtest.h>

#include <memory>
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
    "simple_lane_change_avoidance", node, parameters, rtc_interfaces, marker_interfaces, nullptr};

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

TEST_F(SimpleLaneChangeAvoidanceSceneTest, ObstacleWithoutAdjacentLaneDoesNotRequestExecution)
{
  rclcpp::Node node{"simple_lane_change_avoidance_no_adjacent_lane_test"};
  auto parameters = std::make_shared<SimpleLCAvoidanceParameters>();
  parameters->min_forward_distance = 0.0;
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interfaces;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    marker_interfaces;
  SimpleLaneChangeAvoidanceModule module{
    "simple_lane_change_avoidance", node, parameters, rtc_interfaces, marker_interfaces, nullptr};

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

  ASSERT_NO_THROW(module.run());
  EXPECT_FALSE(module.isExecutionRequested());
}

}  // namespace autoware::behavior_path_planner
