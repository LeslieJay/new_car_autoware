#!/usr/bin/env python3
"""Locate GNSS/EKF pose jumps by comparing both with independent vehicle motion."""

import csv
import math
import os
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Deque, Dict, Optional, Set, Tuple

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


PoseSample = Tuple[int, Pose, float, float]
VelocitySample = Tuple[int, float, float, float]


@dataclass
class MotionResult:
    source: str
    stamp: int
    dt: float
    observed_distance: float
    expected_distance: float
    distance_residual: float
    distance_ratio: float
    vector_residual: float
    observed_dyaw: float
    expected_dyaw: float
    yaw_residual: float
    velocity: VelocitySample
    covariance_x: float
    covariance_y: float
    anomaly: bool
    reason: str


def stamp_ns(msg) -> int:
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_of(pose: Pose) -> float:
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GnssMotionConsistencyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("gnss_motion_consistency_monitor")

        self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("ekf_odom_topic", "/localization/kinematic_state")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("event_topic", "~/jump_detected")
        self.declare_parameter("diagnosis_topic", "~/diagnosis")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("window_sec", 0.20)
        self.declare_parameter("window_tolerance_sec", 0.06)
        self.declare_parameter("pair_tolerance_sec", 0.08)
        self.declare_parameter("max_velocity_age_sec", 0.10)
        self.declare_parameter("position_residual_threshold_m", 0.10)
        # Kept for CLI compatibility; distance_ratio is diagnostic-only now.
        self.declare_parameter("position_ratio_threshold", 2.0)
        self.declare_parameter("yaw_residual_threshold_deg", 3.0)
        self.declare_parameter("trigger_on_yaw", True)
        self.declare_parameter("required_consecutive_windows", 2)
        self.declare_parameter("warning_cooldown_sec", 1.0)
        self.declare_parameter("buffer_duration_sec", 3.0)
        self.declare_parameter("timestamp_reset_threshold_sec", 1.0)

        csv_path = str(self.get_parameter("csv_path").value)
        if not csv_path:
            csv_path = os.path.abspath(
                "gnss_ekf_motion_consistency_"
                + datetime.now().strftime("%Y%m%d_%H%M%S")
                + ".csv"
            )
        csv_directory = os.path.dirname(csv_path)
        if csv_directory:
            os.makedirs(csv_directory, exist_ok=True)
        self.csv_path = csv_path
        self.csv_file = open(csv_path, "w", newline="", buffering=1)
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow([
            "event", "source", "stamp_sec", "dt_sec", "observed_distance_m",
            "expected_distance_m", "distance_residual_m", "distance_ratio",
            "vector_residual_m", "observed_yaw_delta_deg", "expected_yaw_delta_deg",
            "yaw_residual_deg", "longitudinal_velocity_mps", "lateral_velocity_mps",
            "heading_rate_radps", "position_covariance_x", "position_covariance_y",
            "anomaly", "reason", "gnss_anomaly", "ekf_anomaly", "classification",
            "pair_stamp_delta_ms",
        ])

        self.poses: Dict[str, Deque[PoseSample]] = {
            "GNSS": deque(),
            "EKF": deque(),
        }
        self.results: Dict[str, Deque[MotionResult]] = {
            "GNSS": deque(),
            "EKF": deque(),
        }
        self.velocities: Deque[VelocitySample] = deque()
        self.consecutive_anomalies = {"GNSS": 0, "EKF": 0}
        self.evaluated_windows = {"GNSS": 0, "EKF": 0}
        self.classification_counts: Dict[str, int] = {}
        self.classified_gnss_stamps: Set[int] = set()
        self.last_warning_ns = 0
        self.last_timestamp_warning_ns = {"GNSS": 0, "EKF": 0}
        self.duplicate_pose_count = {"GNSS": 0, "EKF": 0}
        self.out_of_order_pose_count = {"GNSS": 0, "EKF": 0}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        gnss_topic = str(self.get_parameter("gnss_pose_topic").value)
        ekf_topic = str(self.get_parameter("ekf_odom_topic").value)
        velocity_topic = str(self.get_parameter("velocity_topic").value)
        self.create_subscription(
            PoseWithCovarianceStamped, gnss_topic, self.on_gnss_pose, qos
        )
        self.create_subscription(Odometry, ekf_topic, self.on_ekf_odometry, qos)
        self.create_subscription(VelocityReport, velocity_topic, self.on_velocity, qos)
        self.event_publisher = self.create_publisher(
            Bool, str(self.get_parameter("event_topic").value), 10
        )
        self.diagnosis_publisher = self.create_publisher(
            String, str(self.get_parameter("diagnosis_topic").value), 10
        )

        self.get_logger().info(
            f"Monitoring GNSS={gnss_topic}, EKF={ekf_topic}, velocity={velocity_topic}; "
            f"position vector threshold="
            f"{float(self.get_parameter('position_residual_threshold_m').value):.3f}m, "
            f"yaw threshold="
            f"{float(self.get_parameter('yaw_residual_threshold_deg').value):.2f}deg; "
            f"CSV={self.csv_path}"
        )

    def on_velocity(self, msg: VelocityReport) -> None:
        stamp = stamp_ns(msg)
        if stamp <= 0:
            stamp = self.get_clock().now().nanoseconds
        self.velocities.append((
            stamp,
            float(msg.longitudinal_velocity),
            float(msg.lateral_velocity),
            float(msg.heading_rate),
        ))
        self.trim(stamp)

    def on_gnss_pose(self, msg: PoseWithCovarianceStamped) -> None:
        covariance = msg.pose.covariance
        self.on_pose("GNSS", stamp_ns(msg), msg.pose.pose, covariance[0], covariance[7])

    def on_ekf_odometry(self, msg: Odometry) -> None:
        covariance = msg.pose.covariance
        self.on_pose("EKF", stamp_ns(msg), msg.pose.pose, covariance[0], covariance[7])

    def on_pose(
        self, source: str, stamp: int, pose: Pose, covariance_x: float, covariance_y: float
    ) -> None:
        if stamp <= 0:
            self.get_logger().warning(f"Ignoring {source} pose with a zero timestamp")
            return
        history = self.poses[source]
        if not history:
            self.consecutive_anomalies[source] = 0
        if history and stamp == history[-1][0]:
            # Some GNSS drivers republish the latest fix faster than the receiver update rate.
            # A repeated timestamp is not a new motion sample and must not reset the window.
            self.duplicate_pose_count[source] += 1
            return
        if history and stamp < history[-1][0]:
            backwards_ns = history[-1][0] - stamp
            reset_threshold_ns = int(
                float(self.get_parameter("timestamp_reset_threshold_sec").value) * 1e9
            )
            if backwards_ns < reset_threshold_ns:
                # DDS callbacks from multiple publishers can arrive slightly out of order.
                # Dropping the stale sample preserves a time-ordered integration window.
                self.out_of_order_pose_count[source] += 1
                self.warn_timestamp_issue(
                    source,
                    f"out-of-order pose dropped ({backwards_ns / 1e6:.1f} ms old)",
                )
                return
            self.warn_timestamp_issue(
                source,
                f"clock moved backwards by {backwards_ns / 1e9:.3f} s; clearing history",
            )
            history.clear()
            self.results[source].clear()
            self.consecutive_anomalies[source] = 0
            if source == "GNSS":
                self.classified_gnss_stamps.clear()
        current = (stamp, pose, covariance_x, covariance_y)
        history.append(current)
        self.trim(stamp)
        result = self.evaluate(source, current)
        if result is not None:
            self.results[source].append(result)
            self.write_sample(result)
            self.try_classify()

    def warn_timestamp_issue(self, source: str, text: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        # Timestamp faults can occur at sensor frequency; throttle identical diagnostics.
        if now_ns - self.last_timestamp_warning_ns[source] >= 1_000_000_000:
            self.last_timestamp_warning_ns[source] = now_ns
            self.get_logger().warning(f"{source} {text}")

    def trim(self, newest_stamp: int) -> None:
        cutoff = newest_stamp - int(
            float(self.get_parameter("buffer_duration_sec").value) * 1e9
        )
        for history in self.poses.values():
            while history and history[0][0] < cutoff:
                history.popleft()
        for results in self.results.values():
            while results and results[0].stamp < cutoff:
                results.popleft()
        self.classified_gnss_stamps = {
            stamp for stamp in self.classified_gnss_stamps if stamp >= cutoff
        }
        # Keep one velocity sample before the cutoff for zero-order-hold integration.
        while len(self.velocities) > 1 and self.velocities[1][0] < cutoff:
            self.velocities.popleft()

    def previous_pose(self, source: str, stamp: int) -> Optional[PoseSample]:
        target = stamp - int(float(self.get_parameter("window_sec").value) * 1e9)
        candidates = [sample for sample in self.poses[source] if sample[0] < stamp]
        if not candidates:
            return None
        result = min(candidates, key=lambda sample: abs(sample[0] - target))
        tolerance = int(float(self.get_parameter("window_tolerance_sec").value) * 1e9)
        return result if abs(result[0] - target) <= tolerance else None

    def integrate_motion(
        self, start_ns: int, end_ns: int, start_yaw: float
    ) -> Optional[Tuple[float, float, float, float, VelocitySample]]:
        if not self.velocities:
            return None
        before = [sample for sample in self.velocities if sample[0] <= start_ns]
        if not before:
            return None
        active = before[-1]
        max_age_ns = int(float(self.get_parameter("max_velocity_age_sec").value) * 1e9)
        if start_ns - active[0] > max_age_ns or end_ns - self.velocities[-1][0] > max_age_ns:
            return None

        updates = [sample for sample in self.velocities if start_ns < sample[0] < end_ns]
        cursor = start_ns
        yaw = start_yaw
        dx = 0.0
        dy = 0.0
        path_length = 0.0
        for next_sample in updates + [None]:
            boundary = next_sample[0] if next_sample is not None else end_ns
            dt = (boundary - cursor) / 1e9
            longitudinal, lateral, heading_rate = active[1], active[2], active[3]
            middle_yaw = yaw + 0.5 * heading_rate * dt
            dx += (
                longitudinal * math.cos(middle_yaw) - lateral * math.sin(middle_yaw)
            ) * dt
            dy += (
                longitudinal * math.sin(middle_yaw) + lateral * math.cos(middle_yaw)
            ) * dt
            path_length += math.hypot(longitudinal, lateral) * dt
            yaw += heading_rate * dt
            cursor = boundary
            if next_sample is not None:
                active = next_sample
        return dx, dy, path_length, normalize_angle(yaw - start_yaw), active

    def evaluate(self, source: str, current: PoseSample) -> Optional[MotionResult]:
        previous = self.previous_pose(source, current[0])
        if previous is None:
            return None
        prediction = self.integrate_motion(previous[0], current[0], yaw_of(previous[1]))
        if prediction is None:
            return None

        observed_dx = current[1].position.x - previous[1].position.x
        observed_dy = current[1].position.y - previous[1].position.y
        observed_distance = math.hypot(observed_dx, observed_dy)
        predicted_dx, predicted_dy, expected_distance, expected_dyaw, velocity = prediction
        distance_residual = observed_distance - expected_distance
        vector_residual = math.hypot(observed_dx - predicted_dx, observed_dy - predicted_dy)
        observed_dyaw = normalize_angle(yaw_of(current[1]) - yaw_of(previous[1]))
        yaw_residual = normalize_angle(observed_dyaw - expected_dyaw)
        ratio = observed_distance / max(expected_distance, 0.01)

        # Compare displacement vectors instead of only their lengths. This detects forward,
        # backward and lateral jumps, and does not become insensitive at higher vehicle speed.
        position_bad = vector_residual > float(
            self.get_parameter("position_residual_threshold_m").value
        )
        yaw_bad = (
            bool(self.get_parameter("trigger_on_yaw").value)
            and abs(math.degrees(yaw_residual))
            > float(self.get_parameter("yaw_residual_threshold_deg").value)
        )
        reasons = []
        if position_bad:
            reasons.append("POSITION")
        if yaw_bad:
            reasons.append("YAW")
        candidate = bool(reasons)
        self.consecutive_anomalies[source] = (
            self.consecutive_anomalies[source] + 1 if candidate else 0
        )
        required = max(1, int(self.get_parameter("required_consecutive_windows").value))
        anomaly = candidate and self.consecutive_anomalies[source] >= required
        self.evaluated_windows[source] += 1

        return MotionResult(
            source=source,
            stamp=current[0],
            dt=(current[0] - previous[0]) / 1e9,
            observed_distance=observed_distance,
            expected_distance=expected_distance,
            distance_residual=distance_residual,
            distance_ratio=ratio,
            vector_residual=vector_residual,
            observed_dyaw=observed_dyaw,
            expected_dyaw=expected_dyaw,
            yaw_residual=yaw_residual,
            velocity=velocity,
            covariance_x=current[2],
            covariance_y=current[3],
            anomaly=anomaly,
            reason="+".join(reasons),
        )

    def write_sample(self, result: MotionResult) -> None:
        self.csv.writerow([
            "SAMPLE", result.source, result.stamp / 1e9, result.dt,
            result.observed_distance, result.expected_distance,
            result.distance_residual, result.distance_ratio, result.vector_residual,
            math.degrees(result.observed_dyaw), math.degrees(result.expected_dyaw),
            math.degrees(result.yaw_residual), result.velocity[1], result.velocity[2],
            result.velocity[3], result.covariance_x, result.covariance_y,
            int(result.anomaly), result.reason, "", "", "", "",
        ])

    def try_classify(self) -> None:
        if not self.results["GNSS"] or not self.results["EKF"]:
            return
        newest_ekf_stamp = self.results["EKF"][-1].stamp
        tolerance_ns = int(float(self.get_parameter("pair_tolerance_sec").value) * 1e9)
        for gnss in self.results["GNSS"]:
            if gnss.stamp in self.classified_gnss_stamps or gnss.stamp > newest_ekf_stamp:
                continue
            ekf = min(self.results["EKF"], key=lambda item: abs(item.stamp - gnss.stamp))
            age_ns = abs(ekf.stamp - gnss.stamp)
            if age_ns > tolerance_ns:
                continue
            self.classified_gnss_stamps.add(gnss.stamp)
            self.classify_pair(gnss, ekf, age_ns)

    def classify_pair(self, gnss: MotionResult, ekf: MotionResult, age_ns: int) -> None:
        if gnss.anomaly and ekf.anomaly:
            classification = "GNSS_INPUT_PROPAGATED_TO_EKF"
        elif gnss.anomaly:
            classification = "GNSS_INPUT_REJECTED_BY_EKF"
        elif ekf.anomaly:
            classification = "EKF_OR_FUSION_OUTPUT_ANOMALY"
        else:
            classification = "NORMAL"
        self.classification_counts[classification] = (
            self.classification_counts.get(classification, 0) + 1
        )
        abnormal = classification != "NORMAL"
        self.csv.writerow([
            "DIAGNOSIS", "GNSS+EKF", gnss.stamp / 1e9,
            "", "", "", "", "", "", "", "", "", "", "", "", "", "",
            int(abnormal), "", int(gnss.anomaly), int(ekf.anomaly), classification,
            age_ns / 1e6,
        ])
        self.event_publisher.publish(Bool(data=abnormal))
        self.diagnosis_publisher.publish(String(data=classification))

        cooldown_ns = int(float(self.get_parameter("warning_cooldown_sec").value) * 1e9)
        if abnormal and gnss.stamp - self.last_warning_ns >= cooldown_ns:
            self.last_warning_ns = gnss.stamp
            self.get_logger().error(
                f"Diagnosis={classification}, pair_age={age_ns/1e6:.1f}ms; "
                f"GNSS[anomaly={gnss.anomaly}, reason={gnss.reason or 'NONE'}, "
                f"pose={gnss.observed_distance:.3f}m, expected={gnss.expected_distance:.3f}m, "
                f"yaw_residual={math.degrees(gnss.yaw_residual):+.2f}deg]; "
                f"EKF[anomaly={ekf.anomaly}, reason={ekf.reason or 'NONE'}, "
                f"pose={ekf.observed_distance:.3f}m, expected={ekf.expected_distance:.3f}m, "
                f"yaw_residual={math.degrees(ekf.yaw_residual):+.2f}deg]"
            )

    def close(self) -> None:
        if not self.csv_file.closed:
            self.csv_file.close()
        # SIGINT may invalidate rosout before spin() returns. Plain stdout keeps the shutdown
        # summary without producing a misleading "publisher's context is invalid" error.
        print(
            f"Stopped: GNSS windows={self.evaluated_windows['GNSS']}, "
            f"EKF windows={self.evaluated_windows['EKF']}, "
            f"duplicate poses={self.duplicate_pose_count}, "
            f"out-of-order poses={self.out_of_order_pose_count}, "
            f"classifications={self.classification_counts}, CSV={self.csv_path}",
            flush=True,
        )


def main() -> None:
    rclpy.init()
    node = GnssMotionConsistencyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
