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

// SPDX-License-Identifier: Apache-2.0
#include "opposite_lane.hpp"
#include "borrowing_math.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <memory>
#include <limits>

namespace autoware::behavior_path_planner
{
class OppositeLaneBorrowingTestPeer
{
public:
  static double returnEndX(const OppositeLaneBorrowing & planner) { return planner.lines_.back().end.position.x; }
  static double selectedSpeed(const OppositeLaneBorrowing & planner) { return planner.candidate_speed_; }
  static void routeFixture(OppositeLaneBorrowing & planner, std::shared_ptr<PlannerData> data)
  {
    lanelet::LineString3d shared(1001, {lanelet::Point3d(1101,-15,2,0), lanelet::Point3d(1102,120,2,0)});
    lanelet::LineString3d right(1002, {lanelet::Point3d(1103,-15,-2,0), lanelet::Point3d(1104,120,-2,0)});
    lanelet::LineString3d far(1003, {lanelet::Point3d(1105,-15,6,0), lanelet::Point3d(1106,120,6,0)});
    shared.attributes()["type"] = "line_thin";
    shared.attributes()["subtype"] = "solid";
    lanelet::Lanelet source(1301, shared, right);
    lanelet::Lanelet opposite(1302, shared.invert(), far.invert());
    for (auto * lane : {&source, &opposite}) {
      lane->attributes()["subtype"] = "road";
      lane->attributes()["location"] = "urban";
      lane->attributes()["one_way"] = "yes";
      lane->attributes()["speed_limit"] = "10";
    }
    auto map = std::make_shared<lanelet::LaneletMap>();
    map->add(source);
    map->add(opposite);
    autoware_map_msgs::msg::LaneletMapBin binary;
    lanelet::utils::conversion::toBinMsg(map, &binary);
    data->route_handler->setMap(binary);
    autoware_planning_msgs::msg::LaneletRoute route;
    route.header.frame_id = "map";
    route.uuid.uuid[0] = 1;
    route.goal_pose.position.x = 100.0;
    route.goal_pose.orientation.w = 1.0;
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = source.id();
    segment.preferred_primitive.primitive_type = "lane";
    segment.primitives = {segment.preferred_primitive};
    route.segments = {segment};
    data->route_handler->setRoute(route);
    data->prev_route_id = route.uuid;
    planner.upstream_.path.points.clear();
    for (int i=-40; i<=440; ++i) {
      PathPointWithLaneId point;
      point.point.pose.position.x = i*0.25;
      point.point.pose.orientation.w = 1.0;
      point.point.longitudinal_velocity_mps = 2.0;
      point.lane_ids = {source.id()};
      planner.upstream_.path.points.push_back(point);
    }
    planner.upstream_.path.header.frame_id = "map";
    planner.upstream_.reference_path = planner.upstream_.path;
  }
  static BehaviorModuleOutput upstream(const OppositeLaneBorrowing & planner) { return planner.upstream_; }
  static bool updateHistory(OppositeLaneBorrowing & planner) { return planner.updateActualHistory(); }
  static void observe(OppositeLaneBorrowing & planner) { planner.updateObjectHistory(); }
  static void clearObjectHistory(OppositeLaneBorrowing & planner) { planner.object_tracks_.clear(); }
  static bool stationary(const OppositeLaneBorrowing & planner, const PredictedObject & object)
  { return planner.isStaticObject(object); }
  static void candidateVelocity(OppositeLaneBorrowing & planner, double speed)
  { planner.candidate_speed_ = speed; planner.applyCandidateVelocity(planner.path_); }
  static void initialize(OppositeLaneBorrowing & planner, const std::shared_ptr<PlannerData> & data)
  {
    planner.data_ = data;
    lanelet::LineString3d left(
      101, {lanelet::Point3d{201, -15.0, 2.0, 0.0}, lanelet::Point3d{202, 40.0, 2.0, 0.0}});
    lanelet::LineString3d right(
      102, {lanelet::Point3d{203, -15.0, -2.0, 0.0}, lanelet::Point3d{204, 40.0, -2.0, 0.0}});
    lanelet::LineString3d far(
      103, {lanelet::Point3d{205, -15.0, 6.0, 0.0}, lanelet::Point3d{206, 40.0, 6.0, 0.0}});
    lanelet::Lanelet source(301, left, right);
    lanelet::Lanelet aligned(302, far, left);
    planner.source_ = {source};
    planner.opposite_ = {aligned.invert()};
    DrivableLanes lanes;
    lanes.left_lane = aligned;
    lanes.right_lane = source;
    planner.drivable_lanes_ = {lanes};
    planner.left_ = true;
    for (int i = -40; i <= 80; ++i) {
      PathPointWithLaneId point;
      point.point.pose.position.x = i * 0.25;
      point.point.pose.orientation.w = 1.0;
      point.point.longitudinal_velocity_mps = 2.0;
      point.lane_ids = {source.id()};
      planner.reference_.points.push_back(point);
    }
    planner.path_.path = planner.reference_;
    planner.path_.shift_length.resize(planner.reference_.points.size(), 0.0);
    planner.upstream_.path = planner.reference_;
    planner.upstream_.reference_path = planner.reference_;
    planner.actual_history_.push_back(data->self_odometry->pose.pose);
  }
  static bool validate(OppositeLaneBorrowing & planner) { return planner.validate(planner.path_); }
  static bool extendReturn(OppositeLaneBorrowing & planner) { return planner.extendReturn(); }
  static double originalReturnEnd(const OppositeLaneBorrowing & planner)
  {
    return planner.original_return_end_arc_;
  }
  static double extendedReturnEnd(const OppositeLaneBorrowing & planner)
  {
    return planner.extended_return_end_arc_;
  }
  static const std::string & extensionConflict(const OppositeLaneBorrowing & planner)
  {
    return planner.return_extension_conflict_;
  }
  static std::string secondaryObjectId(const OppositeLaneBorrowing & planner)
  {
    return autoware_utils_uuid::to_hex_string(
      planner.data_->dynamic_object->objects.front().object_id);
  }
  static void setReturningScenario(OppositeLaneBorrowing & planner)
  {
    setBorrowedReturnScenario(planner);
    auto data = std::const_pointer_cast<PlannerData>(planner.data_);
    auto odometry = std::const_pointer_cast<nav_msgs::msg::Odometry>(data->self_odometry);
    odometry->pose.pose.position.x = 13.0;
    odometry->pose.pose.position.y = 2.5;
    odometry->pose.pose.orientation.z = -std::sin(0.05);
    odometry->pose.pose.orientation.w = std::cos(0.05);
    planner.actual_history_ = {odometry->pose.pose};
    auto objects = std::const_pointer_cast<autoware_perception_msgs::msg::PredictedObjects>(data->dynamic_object);
    objects->objects.front().kinematics.initial_pose_with_covariance.pose.position.x = 20.0;
    objects->objects.front().kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
    planner.candidate_speed_ = 1.0;
    const auto stamp = planner.clock_->now();
    objects->header.stamp = stamp - rclcpp::Duration::from_seconds(1.2);
    planner.updateObjectHistory();
    objects->header.stamp = stamp - rclcpp::Duration::from_seconds(0.6);
    planner.updateObjectHistory();
    objects->header.stamp = stamp;
    planner.updateObjectHistory();
  }
  static void setBorrowedReturnScenario(OppositeLaneBorrowing & planner)
  {
    auto mutable_data = std::const_pointer_cast<PlannerData>(planner.data_);
    auto mutable_odometry = std::const_pointer_cast<nav_msgs::msg::Odometry>(
      mutable_data->self_odometry);
    auto & ego = mutable_odometry->pose.pose;
    ego.position.x = 10.0;
    ego.position.y = 3.0;
    ego.orientation.w = 1.0;
    planner.actual_history_ = {ego};
    for (int i = 81; i <= 152; ++i) {
      PathPointWithLaneId point;
      point.point.pose.position.x = i * 0.25;
      point.point.pose.orientation.w = 1.0;
      point.point.longitudinal_velocity_mps = 2.0;
      point.lane_ids = {planner.source_.front().id()};
      planner.reference_.points.push_back(point);
    }
    for (auto & point : planner.reference_.points) {
      point.point.pose.position.y = 0.0;
    }
    planner.path_.path = planner.reference_;
    planner.path_.shift_length.assign(planner.reference_.points.size(), 3.0);
    for (auto & point : planner.path_.path.points) point.point.pose.position.y = 3.0;
    ShiftLine outbound;
    outbound.start.position.x = 2.0;
    outbound.end.position.x = 6.0;
    outbound.end_shift_length = 3.0;
    ShiftLine returning;
    returning.start.position.x = 12.0;
    returning.end.position.x = 16.0;
    returning.start_shift_length = 3.0;
    planner.lines_ = {outbound, returning};
    planner.original_return_end_arc_ = 26.0;
    planner.extended_return_end_arc_ = 26.0;
    auto object = PredictedObject{};
    object.object_id.uuid[0] = 2;
    object.kinematics.initial_pose_with_covariance.pose.position.x = 15.5;
    object.kinematics.initial_pose_with_covariance.pose.position.y = 1.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = object.shape.dimensions.y = 1.0;
    auto objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
    objects->objects = {object};
    objects->header.stamp = planner.clock_->now();
    mutable_data->dynamic_object = objects;
    planner.conflict_object_ = autoware_utils_uuid::to_hex_string(object.object_id);
    planner.deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  }
  static ShiftedPath & path(OppositeLaneBorrowing & planner) { return planner.path_; }
  static void shift(OppositeLaneBorrowing & planner, double offset, bool return_to_source)
  {
    for (size_t i = 0; i < planner.path_.path.points.size(); ++i) {
      const double x = planner.path_.path.points[i].point.pose.position.x;
      const double fraction = x <= 2.0 ? 0.0 : x < 6.0 ? (x - 2.0) / 4.0 :
        !return_to_source || x < 12.0 ? 1.0 : x < 16.0 ? (16.0 - x) / 4.0 : 0.0;
      planner.path_.path.points[i].point.pose.position.y = offset * fraction;
      planner.path_.shift_length[i] = offset * fraction;
    }
  }
  static void commit(OppositeLaneBorrowing & planner, bool safe)
  {
    planner.requested_ = planner.committed_ = true;
    planner.safe_ = safe;
    planner.phase_ = BorrowingPhase::STOPPING;
  }
  static void trailers(OppositeLaneBorrowing & planner, const TrailerGeometry & geometry)
  {
    planner.active_configuration_.geometries = {geometry};
  }
  static void configure(OppositeLaneBorrowing & planner, const std::vector<std::string> & types)
  {
    byd_vehicle_msgs::msg::TrailerConfiguration message;
    message.trailer_types = types;
    planner.configure(message);
  }
  static bool configurationValid(const OppositeLaneBorrowing & planner)
  {
    return planner.configuration_received_ && planner.configuration_valid_;
  }
  static size_t revision(const OppositeLaneBorrowing & planner)
  {
    return planner.configuration_revision_;
  }
  static std::vector<std::string> configuredTypes(const OppositeLaneBorrowing & planner)
  {
    return planner.configuration_.types;
  }
  static void readyToFinish(OppositeLaneBorrowing & planner)
  {
    planner.committed_ = planner.safe_ = true;
    planner.phase_ = BorrowingPhase::RETURNING;
    ShiftLine line;
    line.end = planner.data_->self_odometry->pose.pose;
    planner.lines_ = {line};
  }
  static void updatePhase(OppositeLaneBorrowing & planner) { planner.updatePhase(); }
  static void prohibitCrossing(OppositeLaneBorrowing & planner)
  {
    planner.drivable_lanes_[0].left_lane = planner.source_[0];
  }
  static void mirrorToRight(OppositeLaneBorrowing & planner)
  {
    lanelet::LineString3d left(
      401, {lanelet::Point3d{501, -15.0, -2.0, 0.0}, lanelet::Point3d{502, 40.0, -2.0, 0.0}});
    lanelet::LineString3d right(
      402, {lanelet::Point3d{503, -15.0, -6.0, 0.0}, lanelet::Point3d{504, 40.0, -6.0, 0.0}});
    lanelet::Lanelet aligned(601, left, right);
    planner.opposite_ = {aligned.invert()};
    planner.drivable_lanes_[0].left_lane = planner.source_[0];
    planner.drivable_lanes_[0].right_lane = aligned;
    planner.left_ = false;
  }
  static void removeOppositeMiddle(OppositeLaneBorrowing & planner)
  {
    lanelet::LineString3d left1(
      701, {lanelet::Point3d{801, -15.0, 6.0, 0.0}, lanelet::Point3d{802, 8.0, 6.0, 0.0}});
    lanelet::LineString3d right1(
      702, {lanelet::Point3d{803, -15.0, 2.0, 0.0}, lanelet::Point3d{804, 8.0, 2.0, 0.0}});
    lanelet::LineString3d left2(
      703, {lanelet::Point3d{805, 12.0, 6.0, 0.0}, lanelet::Point3d{806, 40.0, 6.0, 0.0}});
    lanelet::LineString3d right2(
      704, {lanelet::Point3d{807, 12.0, 2.0, 0.0}, lanelet::Point3d{808, 40.0, 2.0, 0.0}});
    lanelet::Lanelet first(901, left1, right1);
    lanelet::Lanelet last(902, left2, right2);
    planner.opposite_ = {first.invert(), last.invert()};
  }
  static void holdTarget(OppositeLaneBorrowing & planner, const PredictedObject & object)
  {
    planner.target_ = ObjectData{};
    planner.target_->object = object;
  }
};

class OppositeLaneTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_opposite_lane_borrowing");
    parameters = std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{});
    lc = std::make_shared<LaneChangeParameters>();
    data = std::make_shared<PlannerData>();
    odometry = std::make_shared<nav_msgs::msg::Odometry>();
    odometry->pose.pose.orientation.w = 1.0;
    odometry->twist.twist.linear.x = 1.0;
    data->self_odometry = odometry;
    objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
    data->dynamic_object = objects;
    objects->header.stamp = node->now();
    data->parameters.vehicle_info.max_longitudinal_offset_m = 1.0;
    data->parameters.vehicle_info.rear_overhang_m = 0.5;
    data->parameters.vehicle_info.vehicle_width_m = 1.0;
    data->parameters.ego_nearest_dist_threshold = 3.0;
    data->parameters.ego_nearest_yaw_threshold = 1.57;
    planner = std::make_unique<OppositeLaneBorrowing>(*node, parameters, lc);
    OppositeLaneBorrowingTestPeer::initialize(*planner, data);
  }
  PredictedObject obstacle(double x, double y)
  {
    PredictedObject object;
    object.object_id.uuid[0] = 1;
    object.kinematics.initial_pose_with_covariance.pose.position.x = x;
    object.kinematics.initial_pose_with_covariance.pose.position.y = y;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = object.shape.dimensions.y = 1.0;
    return object;
  }
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<AvoidanceByLCParameters> parameters;
  std::shared_ptr<LaneChangeParameters> lc;
  std::shared_ptr<PlannerData> data;
  std::shared_ptr<nav_msgs::msg::Odometry> odometry;
  std::shared_ptr<autoware_perception_msgs::msg::PredictedObjects> objects;
  std::unique_ptr<OppositeLaneBorrowing> planner;
};

TEST_F(OppositeLaneTest, AcceptsEmptyContinuousCorridor)
{
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::validate(*planner)) << planner->reason();
  OppositeLaneBorrowingTestPeer::shift(*planner, 3.0, true);
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::validate(*planner)) << planner->reason();
}

TEST_F(OppositeLaneTest, SupportsRightBorrowingWithoutChangingRawMapDirection)
{
  OppositeLaneBorrowingTestPeer::mirrorToRight(*planner);
  OppositeLaneBorrowingTestPeer::shift(*planner, -3.0, true);
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::validate(*planner)) << planner->reason();
}

TEST_F(OppositeLaneTest, RejectsProhibitedBoundaryAndDisconnectedOppositeLane)
{
  OppositeLaneBorrowingTestPeer::shift(*planner, 3.0, true);
  OppositeLaneBorrowingTestPeer::removeOppositeMiddle(*planner);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "footprint_out_of_corridor");
  OppositeLaneBorrowingTestPeer::prohibitCrossing(*planner);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "discontinuous_or_prohibited_borrowing_section");
}

TEST_F(OppositeLaneTest, RejectsInvalidPredictionsInsteadOfTreatingLaneAsClear)
{
  auto object = obstacle(10.0, 8.0);
  object.kinematics.initial_twist_with_covariance.twist.linear.x =
    std::numeric_limits<double>::quiet_NaN();
  objects->objects.push_back(object);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "invalid_object_prediction");
}

TEST_F(OppositeLaneTest, RejectsTractorCurvatureAndLateralJerkViolation)
{
  data->parameters.vehicle_info.wheel_base_m = 1.0;
  data->parameters.vehicle_info.max_steer_angle_rad = 0.5;
  OppositeLaneBorrowingTestPeer::shift(*planner, 3.0, true);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "kinematic_limit");
}

TEST_F(OppositeLaneTest, RejectsStaticObstacleAndRetainsLostTarget)
{
  const auto object = obstacle(10.0, 0.0);
  objects->objects.push_back(object);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "vehicle_collision");
  objects->objects.clear();
  OppositeLaneBorrowingTestPeer::holdTarget(*planner, object);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "held_target_collision");
}

TEST_F(OppositeLaneTest, DelaysReturnBeyondNewStaticObstacle)
{
  OppositeLaneBorrowingTestPeer::setBorrowedReturnScenario(*planner);
  lc->trajectory.lateral_jerk = 0.5;
  parameters->opposite_lane.max_return_extension_distance = 20.0;
  parameters->opposite_lane.return_extension_step = 1.0;
  ASSERT_TRUE(OppositeLaneBorrowingTestPeer::extendReturn(*planner)) << planner->reason();
  EXPECT_GT(OppositeLaneBorrowingTestPeer::extendedReturnEnd(*planner),
    OppositeLaneBorrowingTestPeer::originalReturnEnd(*planner));
  EXPECT_EQ(OppositeLaneBorrowingTestPeer::extensionConflict(*planner),
    OppositeLaneBorrowingTestPeer::secondaryObjectId(*planner));
  EXPECT_EQ(planner->reason(), "return_extended_for_obstacle");
}

TEST_F(OppositeLaneTest, ChecksObjectInitiallyOutsideBorrowedLane)
{
  auto object = obstacle(10.0, 8.0);
  object.kinematics.initial_twist_with_covariance.twist.linear.y = -1.0;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  objects->objects.push_back(object);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "vehicle_collision");
}

TEST_F(OppositeLaneTest, RejectsTruncatedPredictionAndUnreachableStop)
{
  parameters->opposite_lane.max_prediction_horizon = 5.0;
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "prediction_does_not_cover_complete_return");
  parameters->opposite_lane.max_prediction_horizon = 30.0;
  OppositeLaneBorrowingTestPeer::path(*planner).path.points[60].point.longitudinal_velocity_mps = 0.0;
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
}

TEST_F(OppositeLaneTest, RejectsBoundaryViolationAndMissingReturn)
{
  OppositeLaneBorrowingTestPeer::shift(*planner, 7.0, true);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "footprint_out_of_corridor");
  OppositeLaneBorrowingTestPeer::shift(*planner, 3.0, false);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "last_trailer_not_returned");
}

TEST_F(OppositeLaneTest, InvalidConfigurationDoesNotBecomeAnUnhitchedTractor)
{
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::configurationValid(*planner));
  EXPECT_EQ(OppositeLaneBorrowingTestPeer::revision(*planner), 1U);
  OppositeLaneBorrowingTestPeer::configure(*planner, {});
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::configurationValid(*planner));
  const auto revision = OppositeLaneBorrowingTestPeer::revision(*planner);
  OppositeLaneBorrowingTestPeer::configure(*planner, {"unknown"});
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::configurationValid(*planner));
  EXPECT_GT(OppositeLaneBorrowingTestPeer::revision(*planner), revision);
}

TEST_F(OppositeLaneTest, StaticTrailerConfigurationIsUsedUntilVehicleMessageArrives)
{
  parameters->opposite_lane.trailer_count = 1;
  parameters->opposite_lane.trailer_ordered_types = {"default"};
  TrailerGeometry geometry;
  geometry.type = "default";
  geometry.width = 1.305;
  geometry.axle_to_body_front = 1.0;
  geometry.axle_to_body_rear = 0.7;
  geometry.front_hitch_to_axle = 0.9;
  geometry.axle_to_rear_hitch = 0.6;
  geometry.max_articulation_angle_rad = 0.7853981633974483;
  parameters->opposite_lane.trailer_types.emplace("default", geometry);
  auto configured = std::make_unique<OppositeLaneBorrowing>(*node, parameters, lc);
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::configurationValid(*configured));
  EXPECT_EQ(OppositeLaneBorrowingTestPeer::configuredTypes(*configured),
            std::vector<std::string>{"default"});
  OppositeLaneBorrowingTestPeer::configure(*configured, {});
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::configuredTypes(*configured).empty());
}

TEST_F(OppositeLaneTest, BrakesWithoutDroppingCommittedGeometry)
{
  OppositeLaneBorrowingTestPeer::shift(*planner, 3.0, true);
  const auto original = OppositeLaneBorrowingTestPeer::path(*planner).path;
  OppositeLaneBorrowingTestPeer::commit(*planner, false);
  const auto output = planner->output(false);
  ASSERT_EQ(output.path.points.size(), original.points.size());
  for (size_t i = 0; i < output.path.points.size(); ++i) {
    EXPECT_EQ(output.path.points[i].point.pose.position, original.points[i].point.pose.position);
  }
  EXPECT_EQ(output.path.points.back().point.longitudinal_velocity_mps, 0.0);
  EXPECT_TRUE(planner->committed());
  EXPECT_FALSE(planner->complete());
}

TEST_F(OppositeLaneTest, ApprovalWaitAfterCommitBrakesOnCommittedPath)
{
  OppositeLaneBorrowingTestPeer::shift(*planner, 3.0, true);
  const auto original = OppositeLaneBorrowingTestPeer::path(*planner).path;
  OppositeLaneBorrowingTestPeer::commit(*planner, true);
  const auto output = planner->output(true);
  ASSERT_EQ(output.path.points.size(), original.points.size());
  for (size_t i=0; i<original.points.size(); ++i) {
    EXPECT_EQ(output.path.points[i].point.pose.position, original.points[i].point.pose.position);
  }
  EXPECT_DOUBLE_EQ(output.path.points.back().point.longitudinal_velocity_mps, 0.0);
  EXPECT_TRUE(planner->committed());
}

TEST_F(OppositeLaneTest, IncludesTrailerFootprint)
{
  TrailerGeometry geometry;
  geometry.width = 5.0;  // Tractor fits; this trailer cannot fit in the source lane.
  geometry.axle_to_body_front = 1.0;
  geometry.axle_to_body_rear = 0.7;
  geometry.front_hitch_to_axle = 0.9;
  geometry.axle_to_rear_hitch = 0.6;
  geometry.max_articulation_angle_rad = 0.8;
  OppositeLaneBorrowingTestPeer::trailers(*planner, geometry);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "footprint_out_of_corridor");
}
TEST_F(OppositeLaneTest, ValidTrailerFitsStraightPathAndRejectsArticulationLimit)
{
  TrailerGeometry geometry;
  geometry.width = 1.0;
  geometry.axle_to_body_front = 1.0;
  geometry.axle_to_body_rear = 0.7;
  geometry.front_hitch_to_axle = 0.9;
  geometry.axle_to_rear_hitch = 0.6;
  geometry.max_articulation_angle_rad = 0.8;
  OppositeLaneBorrowingTestPeer::trailers(*planner, geometry);
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::validate(*planner)) << planner->reason();
  geometry.max_articulation_angle_rad = 0.01;
  OppositeLaneBorrowingTestPeer::trailers(*planner, geometry);
  OppositeLaneBorrowingTestPeer::shift(*planner, 3.0, true);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "articulation_limit");
}

TEST_F(OppositeLaneTest, SameDirectionPreferenceAndDisabledFeatureDoNotRequestBorrowing)
{
  ObjectData target(obstacle(10.0, 0.0), 0.0, 10.0, 1.0);
  BehaviorModuleOutput upstream;
  upstream.path = OppositeLaneBorrowingTestPeer::path(*planner).path;
  parameters->opposite_lane.enabled = true;
  planner->update(data, upstream, target, true);
  EXPECT_FALSE(planner->requested());
  EXPECT_FALSE(planner->committed());
  parameters->opposite_lane.enabled = false;
  planner->update(data, upstream, target, false);
  EXPECT_FALSE(planner->requested());
}

TEST_F(OppositeLaneTest, UnsafeForcedExecutionCannotCommit)
{
  const auto output = planner->output(false);
  EXPECT_FALSE(planner->committed());
  EXPECT_EQ(output.path.points.back().point.longitudinal_velocity_mps, 0.0);
}

TEST_F(OppositeLaneTest, CompletionWaitsForTrailerAndStableSamples)
{
  OppositeLaneBorrowingTestPeer::readyToFinish(*planner);
  TrailerGeometry geometry;
  geometry.width = 5.0;
  geometry.axle_to_body_front = 1.0;
  geometry.axle_to_body_rear = 0.7;
  geometry.front_hitch_to_axle = 0.9;
  geometry.axle_to_rear_hitch = 0.6;
  geometry.max_articulation_angle_rad = 0.8;
  OppositeLaneBorrowingTestPeer::trailers(*planner, geometry);
  for (int i = 0; i < 4; ++i) OppositeLaneBorrowingTestPeer::updatePhase(*planner);
  EXPECT_FALSE(planner->complete());
  geometry.width = 1.0;
  OppositeLaneBorrowingTestPeer::trailers(*planner, geometry);
  OppositeLaneBorrowingTestPeer::updatePhase(*planner);
  OppositeLaneBorrowingTestPeer::updatePhase(*planner);
  EXPECT_FALSE(planner->complete());
  OppositeLaneBorrowingTestPeer::updatePhase(*planner);
  EXPECT_TRUE(planner->complete());
}

TEST_F(OppositeLaneTest, CampusUnhitchedHistoryUsesTimestampReachability)
{
  parameters->opposite_lane.campus_agv = true;
  odometry->header.stamp = rclcpp::Time(100, 0);
  ASSERT_TRUE(OppositeLaneBorrowingTestPeer::updateHistory(*planner));
  odometry->pose.pose.position.x = 3.0;
  odometry->header.stamp = rclcpp::Time(102, 0);
  ASSERT_TRUE(OppositeLaneBorrowingTestPeer::updateHistory(*planner));
  odometry->pose.pose.position.x = 10.0;
  odometry->header.stamp = rclcpp::Time(102, 100000000);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::updateHistory(*planner));
}

TEST_F(OppositeLaneTest, CampusStaticHistorySurvivesIdChangeButNotMotion)
{
  parameters->opposite_lane.campus_agv = true;
  objects->objects = {obstacle(10.0, 8.0)};
  for (double stamp : {100.0, 100.5, 101.1}) {
    objects->header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    OppositeLaneBorrowingTestPeer::observe(*planner);
  }
  ASSERT_TRUE(OppositeLaneBorrowingTestPeer::stationary(*planner, objects->objects.front()));
  objects->objects.front().object_id.uuid[0] = 9;
  objects->header.stamp = rclcpp::Time(int64_t{101200000000});
  OppositeLaneBorrowingTestPeer::observe(*planner);
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::stationary(*planner, objects->objects.front()));
  objects->objects.front().kinematics.initial_twist_with_covariance.twist.linear.x = 0.8;
  objects->header.stamp = rclcpp::Time(int64_t{101300000000});
  OppositeLaneBorrowingTestPeer::observe(*planner);
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::stationary(*planner, objects->objects.front()));
}

TEST_F(OppositeLaneTest, CampusStaticLongActionDoesNotNeedPredictionHorizon)
{
  parameters->opposite_lane.campus_agv = true;
  parameters->opposite_lane.max_prediction_horizon = 5.0;
  objects->objects = {obstacle(10.0, 8.0)};
  for (int seconds : {100, 101}) {
    objects->header.stamp = rclcpp::Time(seconds, 0);
    OppositeLaneBorrowingTestPeer::observe(*planner);
  }
  ASSERT_TRUE(OppositeLaneBorrowingTestPeer::stationary(*planner, objects->objects.front()));
  objects->header.stamp = node->now();
  // A confirmed static body uses its measured shape, independent of predicted paths.
  objects->objects.front().kinematics.predicted_paths.emplace_back();
  objects->objects.front().kinematics.predicted_paths.front().path.emplace_back();
  objects->objects.front().kinematics.predicted_paths.front().path.front().position.x =
    std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(OppositeLaneBorrowingTestPeer::validate(*planner)) << planner->reason();
  objects->objects.front().kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "vehicle_collision");
}

TEST_F(OppositeLaneTest, CampusStationaryPedestrianStillRequiresDynamicPrediction)
{
  parameters->opposite_lane.campus_agv = true;
  parameters->opposite_lane.max_prediction_horizon = 5.0;
  objects->objects = {obstacle(10.0, 8.0)};
  autoware_perception_msgs::msg::ObjectClassification label;
  label.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  label.probability = 1.0;
  objects->objects.front().classification = {label};
  for (int seconds : {100, 101}) {
    objects->header.stamp = rclcpp::Time(seconds, 0);
    OppositeLaneBorrowingTestPeer::observe(*planner);
  }
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::stationary(*planner, objects->objects.front()));
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::validate(*planner));
  EXPECT_EQ(planner->reason(), "prediction_does_not_cover_complete_return");
}

TEST_F(OppositeLaneTest, CampusLowerSpeedCandidateRetainsBrakingApproach)
{
  parameters->opposite_lane.campus_agv = true;
  odometry->twist.twist.linear.x = 2.0;
  lc->trajectory.min_longitudinal_acc = -1.0;
  OppositeLaneBorrowingTestPeer::candidateVelocity(*planner, 1.0);
  const auto & points = OppositeLaneBorrowingTestPeer::path(*planner).path.points;
  EXPECT_DOUBLE_EQ(points[40].point.longitudinal_velocity_mps, 2.0);
  EXPECT_DOUBLE_EQ(points[60].point.longitudinal_velocity_mps, 1.0);
}

TEST_F(OppositeLaneTest, CampusUpdateSearchesLowerSpeedAndCommitsCompletePath)
{
  parameters->opposite_lane.enabled = true;
  parameters->opposite_lane.campus_agv = true;
  parameters->opposite_lane.max_prediction_horizon = 60.0;
  parameters->opposite_lane.longitudinal_margin = 1.0;
  parameters->opposite_lane.lateral_margin = 0.3;
  parameters->opposite_lane.max_planning_time_ms = 2000.0;
  data->parameters.vehicle_info.wheel_base_m = 1.0;
  data->parameters.vehicle_info.max_steer_angle_rad = 1.0;
  lc->trajectory.lateral_jerk = 0.5;
  odometry->header.stamp = node->now();
  objects->header.stamp = node->now();
  objects->objects = {obstacle(20.0, 0.0)};
  OppositeLaneBorrowingTestPeer::routeFixture(*planner, data);
  const auto upstream = OppositeLaneBorrowingTestPeer::upstream(*planner);
  ObjectData target(objects->objects.front(), 0.0, 20.0, 1.0);
  target.direction = Direction::RIGHT;
  planner->update(data, upstream, target, false);
  ASSERT_TRUE(planner->ready()) << planner->reason();
  EXPECT_LT(OppositeLaneBorrowingTestPeer::selectedSpeed(*planner), 2.0);
  const auto output = planner->output(false);
  EXPECT_TRUE(planner->committed());
  EXPECT_EQ(planner->phase(), BorrowingPhase::BORROWING);
  EXPECT_GT(output.path.points.back().point.pose.position.x, 30.0);
  EXPECT_GE(output.path.points.back().point.pose.position.x -
    OppositeLaneBorrowingTestPeer::returnEndX(*planner), 3.0);
}

TEST_F(OppositeLaneTest, CampusReturningCandidateStartsAtMeasuredPoseAndHeading)
{
  parameters->opposite_lane.campus_agv = true;
  parameters->opposite_lane.max_prediction_horizon = 60.0;
  parameters->opposite_lane.longitudinal_margin = 1.0;
  parameters->opposite_lane.lateral_margin = 0.3;
  lc->trajectory.lateral_jerk = 0.5;
  OppositeLaneBorrowingTestPeer::setReturningScenario(*planner);
  ASSERT_TRUE(OppositeLaneBorrowingTestPeer::extendReturn(*planner)) << planner->reason();
  const auto & path = OppositeLaneBorrowingTestPeer::path(*planner).path;
  const auto index = data->findEgoIndex(path.points);
  EXPECT_NEAR(path.points[index].point.pose.position.x, 13.0, 1e-6);
  EXPECT_NEAR(path.points[index].point.pose.position.y, 2.5, 1e-6);
  EXPECT_NEAR(path.points[index].point.pose.orientation.z, -std::sin(0.05), 1e-6);
  EXPECT_LE(OppositeLaneBorrowingTestPeer::extendedReturnEnd(*planner),
    OppositeLaneBorrowingTestPeer::originalReturnEnd(*planner) + 20.0 + 1e-6);
}

TEST_F(OppositeLaneTest, CampusUnconfirmedSecondaryRetainsTemporalChecks)
{
  parameters->opposite_lane.campus_agv = true;
  parameters->opposite_lane.longitudinal_margin = 1.0;
  parameters->opposite_lane.lateral_margin = 0.3;
  lc->trajectory.lateral_jerk = 0.5;
  OppositeLaneBorrowingTestPeer::setReturningScenario(*planner);
  OppositeLaneBorrowingTestPeer::clearObjectHistory(*planner);
  parameters->opposite_lane.max_prediction_horizon = 5.0;
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::extendReturn(*planner));
  EXPECT_NE(planner->reason().find("prediction_does_not_cover_complete_return"), std::string::npos);
  parameters->opposite_lane.max_prediction_horizon = 60.0;
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::extendReturn(*planner)) << planner->reason();
}

TEST_F(OppositeLaneTest, CampusReturningFailureRetainsCommittedGeometry)
{
  parameters->opposite_lane.campus_agv = true;
  parameters->opposite_lane.max_return_extension_distance = 1.0;
  OppositeLaneBorrowingTestPeer::setReturningScenario(*planner);
  const auto before = OppositeLaneBorrowingTestPeer::path(*planner).path;
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::extendReturn(*planner));
  const auto & after = OppositeLaneBorrowingTestPeer::path(*planner).path;
  ASSERT_EQ(before.points.size(), after.points.size());
  for (size_t i=0; i<before.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(before.points[i].point.pose.position.y, after.points[i].point.pose.position.y);
  }
}

TEST(CampusLateralSegment, PreservesMeasuredTangentAndCurvature)
{
  const borrowing::LateralSegment curve(-3.0, 0.15, -0.02, 0.0, 25.0);
  EXPECT_NEAR(curve.value(0.0), -3.0, 1e-10);
  EXPECT_NEAR(curve.slope(0.0), 0.15, 1e-10);
  EXPECT_NEAR((curve.slope(0.001) - curve.slope(0.0)) / 0.001, -0.02, 1e-4);
  EXPECT_NEAR(curve.value(25.0), 0.0, 1e-10);
  EXPECT_NEAR(curve.slope(25.0), 0.0, 1e-10);
}

}  // namespace autoware::behavior_path_planner
