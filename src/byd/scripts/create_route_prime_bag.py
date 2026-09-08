#!/usr/bin/env python3

"""Extract a short route-only bag used to initialize planning before a seeked replay."""

import sys
from pathlib import Path

import rosbag2_py
import yaml


ROUTE_TOPICS = {
    "/planning/mission_planning/route",
    "/planning/mission_planning/state",
    "/planning/route",
    "/planning/route_state",
}


def main():
    if len(sys.argv) != 4:
        raise SystemExit("usage: create_route_prime_bag.py SOURCE OUTPUT OFFSET_SECONDS")
    source = Path(sys.argv[1])
    output = Path(sys.argv[2])
    offset_seconds = float(sys.argv[3])

    metadata = yaml.safe_load((source / "metadata.yaml").read_text())
    source_start = int(
        metadata["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"]
    )
    begin = source_start + int(offset_seconds * 1e9)
    end = begin + int(5e9)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    source_topics = {
        topic.name: topic for topic in reader.get_all_topics_and_types()
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(ROUTE_TOPICS)))
    reader.seek(begin)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic_name in ROUTE_TOPICS:
        topic = source_topics[topic_name]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic.name,
                type=topic.type,
                serialization_format=topic.serialization_format,
                offered_qos_profiles=topic.offered_qos_profiles,
            )
        )

    count = 0
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if timestamp > end:
            break
        writer.write(topic, serialized, timestamp)
        count += 1

    if count == 0:
        raise SystemExit("no route messages found in the requested priming window")
    print(f"Created route primer with {count} messages: {output}")


if __name__ == "__main__":
    main()
