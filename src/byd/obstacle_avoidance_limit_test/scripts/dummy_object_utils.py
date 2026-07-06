#!/usr/bin/env python3
# Copyright 2026 BYD
"""Utilities for publishing tier4_simulation_msgs/DummyObject."""

from __future__ import annotations

import math
import uuid
from typing import Literal, Tuple

from autoware_perception_msgs.msg import ObjectClassification, Shape
from geometry_msgs.msg import Pose, Quaternion
from tier4_simulation_msgs.msg import DummyObject


ShoulderSide = Literal["right", "left"]


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def unit_vectors(yaw: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    forward = (math.cos(yaw), math.sin(yaw))
    right = (math.sin(yaw), -math.cos(yaw))
    return forward, right


def compute_obstacle_pose(
    anchor_x: float,
    anchor_y: float,
    lane_yaw: float,
    lane_width: float,
    shoulder_side: ShoulderSide,
    longitudinal_m: float,
    intrusion_m: float,
    obj_width: float,
) -> Tuple[float, float, float]:
    """
    从路肩边缘向车道内侵入 intrusion_m 处放置障碍物中心。

    anchor 为车道中心线上的纵向零点（通常为 ego 位置）。
    """
    forward, right = unit_vectors(lane_yaw)
    half_lane = lane_width / 2.0
    lateral_from_center = half_lane - intrusion_m - obj_width / 2.0

    if shoulder_side == "left":
        lateral_from_center = -lateral_from_center

    ox = anchor_x + forward[0] * longitudinal_m + right[0] * lateral_from_center
    oy = anchor_y + forward[1] * longitudinal_m + right[1] * lateral_from_center
    return ox, oy, lane_yaw


def make_dummy_object(
    frame_id: str,
    stamp,
    x: float,
    y: float,
    z: float,
    yaw: float,
    *,
    length: float = 4.0,
    width: float = 1.8,
    height: float = 2.0,
    velocity: float = 0.0,
    label: int = ObjectClassification.CAR,
    action: int = DummyObject.ADD,
    object_uuid: bytes | None = None,
) -> DummyObject:
    msg = DummyObject()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    if object_uuid is None:
        object_uuid = uuid.uuid4().bytes
    msg.id.uuid = list(object_uuid)

    msg.classification.label = label
    msg.classification.probability = 1.0

    msg.shape.type = Shape.BOUNDING_BOX
    msg.shape.dimensions.x = length
    msg.shape.dimensions.y = width
    msg.shape.dimensions.z = height

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = yaw_to_quaternion(yaw)
    msg.initial_state.pose_covariance.pose = pose

    std = 0.03
    cov = [0.0] * 36
    cov[0] = std * std
    cov[7] = std * std
    cov[14] = std * std
    cov[35] = (5.0 * math.pi / 180.0) ** 2
    msg.initial_state.pose_covariance.covariance = cov

    msg.initial_state.twist_covariance.twist.linear.x = velocity
    msg.initial_state.accel_covariance.accel.linear.x = 0.0

    msg.max_velocity = 33.3
    msg.min_velocity = -33.3
    msg.action = action
    return msg


def make_delete_all(stamp, frame_id: str = "map") -> DummyObject:
    msg = DummyObject()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.action = DummyObject.DELETEALL
    return msg
