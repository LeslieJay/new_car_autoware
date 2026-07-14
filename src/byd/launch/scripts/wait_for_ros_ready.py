#!/usr/bin/env python3
"""Wait until ROS 2 topics, services, or TF transforms are available."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class RosReadinessWaiter(Node):
    def __init__(self) -> None:
        super().__init__("ros_readiness_waiter")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _pause(self, poll_interval: float) -> None:
        # Spin first so graph and TF updates are processed before sleeping.
        rclpy.spin_once(self, timeout_sec=min(0.2, poll_interval))
        remaining = max(0.0, poll_interval - 0.2)
        if remaining:
            time.sleep(remaining)

    def wait_for_topics(
        self,
        topics: list[str],
        timeout_sec: float,
        poll_interval: float,
    ) -> list[str]:
        deadline = time.monotonic() + timeout_sec
        pending = set(topics)

        while pending and time.monotonic() < deadline:
            for topic in list(pending):
                publisher_count = self.count_publishers(topic)
                if publisher_count > 0:
                    self.get_logger().info(
                        f"Topic ready: {topic} ({publisher_count} publishers)"
                    )
                    pending.discard(topic)

            if pending:
                self.get_logger().info(f"Waiting for topics: {sorted(pending)}")
                self._pause(poll_interval)

        return sorted(pending)

    def wait_for_services(
        self,
        services: list[str],
        timeout_sec: float,
        poll_interval: float,
    ) -> list[str]:
        deadline = time.monotonic() + timeout_sec
        pending = set(services)

        while pending and time.monotonic() < deadline:
            available = {name for name, _ in self.get_service_names_and_types()}
            for service in list(pending):
                if service in available:
                    self.get_logger().info(f"Service ready: {service}")
                    pending.discard(service)

            if pending:
                self.get_logger().info(f"Waiting for services: {sorted(pending)}")
                self._pause(poll_interval)

        return sorted(pending)

    def wait_for_tfs(
        self,
        transforms: list[tuple[str, str]],
        timeout_sec: float,
        poll_interval: float,
    ) -> list[tuple[str, str]]:
        deadline = time.monotonic() + timeout_sec
        pending = set(transforms)

        while pending and time.monotonic() < deadline:
            # Time() means "latest available transform".
            latest = rclpy.time.Time()
            for parent, child in list(pending):
                if self._tf_buffer.can_transform(
                    parent,
                    child,
                    latest,
                    timeout=rclpy.duration.Duration(seconds=0.2),
                ):
                    self.get_logger().info(f"TF ready: {parent} -> {child}")
                    pending.discard((parent, child))

            if pending:
                names = [f"{p}->{c}" for p, c in sorted(pending)]
                self.get_logger().info(f"Waiting for TF: {names}")
                self._pause(poll_interval)

        return sorted(pending)


def _parse_tf_arg(value: str) -> tuple[str, str]:
    if ":" not in value:
        raise argparse.ArgumentTypeError(
            f"TF argument must be 'parent:child', got '{value}'"
        )

    parent, child = (part.strip() for part in value.split(":", 1))
    if not parent or not child:
        raise argparse.ArgumentTypeError(
            f"TF parent and child must both be non-empty, got '{value}'"
        )
    return parent, child


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", action="append", default=[], dest="topics")
    parser.add_argument("--service", action="append", default=[], dest="services")
    parser.add_argument(
        "--tf",
        action="append",
        default=[],
        type=_parse_tf_arg,
        dest="tfs",
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--poll-interval", type=float, default=1.0)
    args = parser.parse_args()

    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    if args.poll_interval <= 0:
        parser.error("--poll-interval must be greater than zero")
    if not args.topics and not args.services and not args.tfs:
        parser.error("at least one of --topic, --service, or --tf is required")

    rclpy.init()
    waiter = RosReadinessWaiter()
    failures: list[str] = []

    try:
        if args.topics:
            pending = waiter.wait_for_topics(
                args.topics,
                args.timeout,
                args.poll_interval,
            )
            failures.extend(f"topic:{name}" for name in pending)

        if args.services:
            pending = waiter.wait_for_services(
                args.services,
                args.timeout,
                args.poll_interval,
            )
            failures.extend(f"service:{name}" for name in pending)

        if args.tfs:
            pending = waiter.wait_for_tfs(
                args.tfs,
                args.timeout,
                args.poll_interval,
            )
            failures.extend(f"tf:{parent}->{child}" for parent, child in pending)
    except KeyboardInterrupt:
        return 130
    finally:
        waiter.destroy_node()
        rclpy.shutdown()

    if failures:
        print(
            f"Readiness check timed out after {args.timeout}s: {failures}",
            file=sys.stderr,
        )
        return 1

    print("All readiness checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
