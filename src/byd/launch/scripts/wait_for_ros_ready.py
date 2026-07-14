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
        super().__init__('ros_readiness_waiter')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
                if self.count_publishers(topic) > 0:
                    self.get_logger().info(
                        f'Topic ready: {topic} ({self.count_publishers(topic)} publishers)',
                    )
                    pending.discard(topic)
            if pending:
                self.get_logger().info(f'Waiting for topics: {sorted(pending)}')
                time.sleep(poll_interval)
                rclpy.spin_once(self, timeout_sec=0.1)
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
                    self.get_logger().info(f'Service ready: {service}')
                    pending.discard(service)
            if pending:
                self.get_logger().info(f'Waiting for services: {sorted(pending)}')
                time.sleep(poll_interval)
                rclpy.spin_once(self, timeout_sec=0.1)
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
            now = rclpy.time.Time()
            for parent, child in list(pending):
                if self._tf_buffer.can_transform(parent, child, now, timeout=rclpy.duration.Duration(seconds=0.5)):
                    self.get_logger().info(f'TF ready: {parent} -> {child}')
                    pending.discard((parent, child))
            if pending:
                self.get_logger().info(
                    f'Waiting for TF: {[f"{p}->{c}" for p, c in sorted(pending)]}',
                )
                time.sleep(poll_interval)
                rclpy.spin_once(self, timeout_sec=0.1)
        return sorted(pending)


def _parse_tf_arg(value: str) -> tuple[str, str]:
    if ':' not in value:
        raise argparse.ArgumentTypeError(
            f"TF argument must be 'parent:child', got '{value}'",
        )
    parent, child = value.split(':', 1)
    return parent.strip(), child.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', action='append', default=[], dest='topics')
    parser.add_argument('--service', action='append', default=[], dest='services')
    parser.add_argument('--tf', action='append', default=[], type=_parse_tf_arg, dest='tfs')
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--poll-interval', type=float, default=1.0)
    args = parser.parse_args()

    if not args.topics and not args.services and not args.tfs:
        print('At least one of --topic, --service, or --tf is required', file=sys.stderr)
        return 2

    rclpy.init()
    waiter = RosReadinessWaiter()
    failures: list[str] = []

    try:
        if args.topics:
            pending = waiter.wait_for_topics(args.topics, args.timeout, args.poll_interval)
            if pending:
                failures.extend(f'topic:{name}' for name in pending)

        if args.services:
            pending = waiter.wait_for_services(args.services, args.timeout, args.poll_interval)
            if pending:
                failures.extend(f'service:{name}' for name in pending)

        if args.tfs:
            pending = waiter.wait_for_tfs(args.tfs, args.timeout, args.poll_interval)
            if pending:
                failures.extend(f'tf:{parent}->{child}' for parent, child in pending)
    finally:
        waiter.destroy_node()
        rclpy.shutdown()

    if failures:
        print(f'Readiness check timed out after {args.timeout}s: {failures}', file=sys.stderr)
        return 1

    print('All readiness checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
