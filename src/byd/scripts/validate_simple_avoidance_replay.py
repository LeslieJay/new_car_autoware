#!/usr/bin/env python3

"""Validate that replay did not produce a persistent outside-drivable-area stop."""

import argparse
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


VIRTUAL_WALL_TOPIC = (
    "/planning/scenario_planning/lane_driving/motion_planning/"
    "path_optimizer/virtual_wall"
)
TRAJECTORY_TOPIC = (
    "/planning/scenario_planning/lane_driving/motion_planning/"
    "path_optimizer/trajectory"
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_bag", type=Path)
    parser.add_argument("--source-bag", required=True, type=Path)
    parser.add_argument("--analyze-after-seconds", type=float, default=300.0)
    parser.add_argument("--max-outside-drivable-duration", type=float, default=5.0)
    return parser.parse_args()


def source_start_time(source_bag):
    metadata = yaml.safe_load((source_bag / "metadata.yaml").read_text())
    start_ns = metadata["rosbag2_bagfile_information"]["starting_time"][
        "nanoseconds_since_epoch"
    ]
    return int(start_ns) / 1e9


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def persistent_clusters(timestamps, maximum_gap=0.5):
    if not timestamps:
        return []
    clusters = []
    start = previous = timestamps[0]
    for timestamp in timestamps[1:]:
        if timestamp - previous > maximum_gap:
            clusters.append((start, previous))
            start = timestamp
        previous = timestamp
    clusters.append((start, previous))
    return clusters


def main():
    args = parse_args()
    threshold = source_start_time(args.source_bag) + args.analyze_after_seconds

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.result_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required_topics = {VIRTUAL_WALL_TOPIC, TRAJECTORY_TOPIC}
    missing_topics = required_topics - topic_types.keys()
    if missing_topics:
        raise SystemExit(f"FAIL: result bag is missing topics: {sorted(missing_topics)}")

    message_types = {
        topic: get_message(topic_types[topic]) for topic in required_topics
    }
    outside_drivable_timestamps = set()
    trajectory_count = 0

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in required_topics:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == TRAJECTORY_TOPIC:
            if stamp_seconds(message.header.stamp) >= threshold:
                trajectory_count += 1
            continue
        for marker in message.markers:
            marker_time = stamp_seconds(marker.header.stamp)
            if marker_time >= threshold and "outside drivable area" in marker.text.lower():
                outside_drivable_timestamps.add(marker_time)

    if trajectory_count == 0:
        raise SystemExit("FAIL: no path-optimizer trajectories were recorded after the test event")

    clusters = persistent_clusters(sorted(outside_drivable_timestamps))
    longest_duration = max((end - start for start, end in clusters), default=0.0)
    print(f"  path-optimizer trajectories: {trajectory_count}")
    print(f"  longest outside-drivable cluster: {longest_duration:.3f}s")
    for start, end in clusters:
        duration = end - start
        if duration > args.max_outside_drivable_duration:
            print(
                "  persistent cluster source offsets: "
                f"{start - source_start_time(args.source_bag):.3f}s.."
                f"{end - source_start_time(args.source_bag):.3f}s "
                f"({duration:.3f}s)"
            )
    if longest_duration > args.max_outside_drivable_duration:
        raise SystemExit(
            "FAIL: persistent outside-drivable-area stop remained after lane-change avoidance "
            f"({longest_duration:.3f}s)"
        )
    print("  outside-drivable-area persistence: PASS")


if __name__ == "__main__":
    main()
