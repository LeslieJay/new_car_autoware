#!/usr/bin/env python3

"""Extract a short route-only bag used to initialize planning before a seeked replay."""

import sys
import sqlite3
from pathlib import Path

import rosbag2_py
import yaml


ROUTE_TOPICS = {
    "/planning/mission_planning/route",
    "/planning/mission_planning/state",
    "/planning/route",
    "/planning/route_state",
}


def source_start_time_ns(source: Path) -> int:
    metadata_path = source / "metadata.yaml" if source.is_dir() else None
    if metadata_path and metadata_path.is_file():
        metadata = yaml.safe_load(metadata_path.read_text())
        return int(
            metadata["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"]
        )
    if source.is_file() and source.suffix == ".db3":
        connection = sqlite3.connect(f"file:{source}?mode=ro", uri=True)
        try:
            row = connection.execute("SELECT MIN(timestamp) FROM messages").fetchone()
        finally:
            connection.close()
        if not row or row[0] is None:
            raise SystemExit(f"source bag contains no messages: {source}")
        return int(row[0])
    raise SystemExit(f"expected a rosbag directory with metadata.yaml or a .db3 file: {source}")


def main():
    if len(sys.argv) != 4:
        raise SystemExit("usage: create_route_prime_bag.py SOURCE OUTPUT OFFSET_SECONDS")
    source = Path(sys.argv[1])
    output = Path(sys.argv[2])
    offset_seconds = float(sys.argv[3])

    source_start = source_start_time_ns(source)
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
    available_route_topics = ROUTE_TOPICS & source_topics.keys()
    for topic_name in available_route_topics:
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
