// Copyright 2026 BYD
// SPDX-License-Identifier: Apache-2.0
#include "opposite_lane.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <memory>
#include <limits>

namespace autoware::behavior_path_planner
{
class OppositeLaneBorrowingTestPeer
{
public:
  static void initialize(OppositeLaneBorrowing & planner, const std::shared_ptr<PlannerData> & data)
  {
    planner.data_ = data;
    lanelet::LineString3d left(101, {{201, -15.0, 2.0, 0.0}, {202, 40.0, 2.0, 0.0}});
    lanelet::LineString3d right(102, {{203, -15.0, -2.0, 0.0}, {204, 40.0, -2.0, 0.0}});
    lanelet::LineString3d far(103, {{205, -15.0, 6.0, 0.0}, {206, 40.0, 6.0, 0.0}});
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
    lanelet::LineString3d left(401, {{501, -15.0, -2.0, 0.0}, {502, 40.0, -2.0, 0.0}});
    lanelet::LineString3d right(402, {{503, -15.0, -6.0, 0.0}, {504, 40.0, -6.0, 0.0}});
    lanelet::Lanelet aligned(601, left, right);
    planner.opposite_ = {aligned.invert()};
    planner.drivable_lanes_[0].left_lane = planner.source_[0];
    planner.drivable_lanes_[0].right_lane = aligned;
    planner.left_ = false;
  }
  static void removeOppositeMiddle(OppositeLaneBorrowing & planner)
  {
    lanelet::LineString3d left1(701, {{801, -15.0, 6.0, 0.0}, {802, 8.0, 6.0, 0.0}});
    lanelet::LineString3d right1(702, {{803, -15.0, 2.0, 0.0}, {804, 8.0, 2.0, 0.0}});
    lanelet::LineString3d left2(703, {{805, 12.0, 6.0, 0.0}, {806, 40.0, 6.0, 0.0}});
    lanelet::LineString3d right2(704, {{807, 12.0, 2.0, 0.0}, {808, 40.0, 2.0, 0.0}});
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
    objects->header.stamp = node->now().to_msg();
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
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::configurationValid(*planner));
  OppositeLaneBorrowingTestPeer::configure(*planner, {});
  EXPECT_TRUE(OppositeLaneBorrowingTestPeer::configurationValid(*planner));
  const auto revision = OppositeLaneBorrowingTestPeer::revision(*planner);
  OppositeLaneBorrowingTestPeer::configure(*planner, {"unknown"});
  EXPECT_FALSE(OppositeLaneBorrowingTestPeer::configurationValid(*planner));
  EXPECT_GT(OppositeLaneBorrowingTestPeer::revision(*planner), revision);
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

}  // namespace autoware::behavior_path_planner
