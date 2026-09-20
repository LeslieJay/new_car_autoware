#!/usr/bin/env python3

# Copyright 2026 BYD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import ClearRoute
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


PACKAGE = 'byd_vehicle_state'
NODE_NAME = 'byd_vehicle_state'


class RouteClearHarness:

    def __init__(self):
        self.node = rclpy.create_node('arrival_clears_route_test_harness')
        self.clear_route_calls = 0
        self.forward_goal_pub = self.node.create_publisher(
            PoseStamped, f'/{NODE_NAME}/input/forward_goal', 10)
        self.odom_pub = self.node.create_publisher(
            Odometry, f'/{NODE_NAME}/input/odometry', 10)
        self.route_state_pub = self.node.create_publisher(
            RouteState,
            f'/{NODE_NAME}/input/route_state',
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )
        self.node.create_service(ClearRoute, '/api/routing/clear_route', self._clear_route)

    def _clear_route(self, _request, response):
        self.clear_route_calls += 1
        response.status.success = True
        return response

    def publish_inputs(self, route_state, publish_goal=False):
        now = self.node.get_clock().now().to_msg()

        if publish_goal:
            goal = PoseStamped()
            goal.header.stamp = now
            goal.header.frame_id = 'map'
            goal.pose.orientation.w = 1.0
            self.forward_goal_pub.publish(goal)

        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = 'map'
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 0.0
        self.odom_pub.publish(odometry)

        state = RouteState()
        state.state = route_state
        self.route_state_pub.publish(state)


@pytest.fixture
def running_system():
    rclpy.init()
    harness = RouteClearHarness()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(harness.node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    executable = Path(get_package_prefix(PACKAGE)) / 'lib' / PACKAGE / 'vehicle_state_node_exe'
    process = subprocess.Popen(
        [
            str(executable),
            '--ros-args',
            '-p', 'arrival_check_longitudinal_undershoot_distance:=0.05',
            '-p', 'arrival_check_longitudinal_overshoot_distance:=0.05',
            '-p', 'arrival_check_lateral_distance:=0.05',
            '-p', 'arrival_check_angle_deg:=5.0',
            '-p', 'arrival_check_duration:=0.1',
            '-p', 'arrived_to_unset_timeout:=5.0',
            '-p', 'clear_route_retry_interval:=0.1',
        ],
        env=os.environ.copy(),
    )
    try:
        yield harness
    finally:
        process.terminate()
        process.wait(timeout=10)
        executor.shutdown()
        harness.node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)


def wait_until(predicate, harness, route_state, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        harness.publish_inputs(route_state)
        if predicate():
            return True
        time.sleep(0.03)
    return False


def test_forward_arrival_clears_route(running_system):
    harness = running_system
    assert wait_until(
        lambda: harness.forward_goal_pub.get_subscription_count() > 0,
        harness,
        RouteState.SET,
    )
    harness.publish_inputs(RouteState.SET, publish_goal=True)

    assert wait_until(
        lambda: harness.clear_route_calls == 0,
        harness,
        RouteState.SET,
        timeout=0.2,
    )
    assert wait_until(
        lambda: harness.clear_route_calls >= 1,
        harness,
        RouteState.ARRIVED,
        timeout=5.0,
    )
