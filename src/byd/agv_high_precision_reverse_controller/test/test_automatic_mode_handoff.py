#!/usr/bin/env python3

# Copyright 2026 BYD.
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
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from reverse_parking_planner.srv import SetGoalPose
from tier4_control_msgs.srv import SetPause
from tier4_external_api_msgs.srv import Engage


PACKAGE = 'agv_high_precision_reverse_controller'
NODE_NAME = 'agv_high_precision_reverse_controller'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/agv_high_precision_reverse_controller_test_logs')


class AutowareServiceHarness:

    def __init__(self):
        self.node = rclpy.create_node('automatic_mode_handoff_test_harness')
        self.events = []
        self.local_failures_remaining = 0
        self.odom_pub = self.node.create_publisher(
            Odometry, f'/{NODE_NAME}/input/odometry', 10)
        self.node.create_service(
            SetPause, '/control/vehicle_cmd_gate/set_pause', self._pause)
        self.node.create_service(
            ChangeOperationMode,
            '/api/operation_mode/change_to_local',
            self._local)
        self.node.create_service(ClearRoute, '/api/routing/clear_route', self._clear_route)
        self.node.create_service(
            ChangeOperationMode,
            '/api/operation_mode/enable_autoware_control',
            self._enable)
        self.node.create_service(
            ChangeOperationMode,
            '/api/operation_mode/change_to_autonomous',
            self._autonomous)
        self.node.create_service(Engage, '/api/autoware/set/engage', self._engage)

    def _pause(self, request, response):
        self.events.append(f'pause:{str(request.pause).lower()}')
        response.status.success = True
        return response

    def _local(self, _request, response):
        if self.local_failures_remaining:
            self.local_failures_remaining -= 1
            self.events.append('local:transition')
            response.status.success = False
            response.status.code = response.ERROR_IN_TRANSITION
            response.status.message = 'The mode transition is in progress.'
            return response
        self.events.append('local')
        response.status.success = True
        return response

    def _clear_route(self, _request, response):
        self.events.append('clear_route')
        response.status.success = True
        return response

    def _enable(self, _request, response):
        self.events.append('enable')
        response.status.success = True
        return response

    def _autonomous(self, _request, response):
        self.events.append('autonomous')
        response.status.success = True
        return response

    def _engage(self, _request, response):
        self.events.append('engage')
        response.status.code = response.status.SUCCESS
        return response

    def publish_odometry(self, x=0.0):
        message = Odometry()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(message)


@pytest.fixture
def running_system():
    rclpy.init()
    harness = AutowareServiceHarness()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(harness.node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    executable = (
        Path(get_package_prefix(PACKAGE)) / 'lib' / PACKAGE /
        f'{PACKAGE}_node_exe')
    process = subprocess.Popen(
        [str(executable), '--ros-args', '-p', 'safety.odometry_timeout:=2.0'],
        env=os.environ.copy())
    try:
        yield harness
    finally:
        process.terminate()
        process.wait(timeout=10)
        executor.shutdown()
        harness.node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)


def wait_until(predicate, harness, timeout=5.0, odom_x=0.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        harness.publish_odometry(odom_x)
        if predicate():
            return True
        time.sleep(0.05)
    return False


def test_reverse_goal_automatically_takes_external_control(running_system):
    harness = running_system
    client = harness.node.create_client(
        SetGoalPose, f'/{NODE_NAME}/set_goal_pose')
    assert client.wait_for_service(timeout_sec=5.0)
    assert wait_until(lambda: harness.odom_pub.get_subscription_count() > 0, harness)

    harness.publish_odometry()
    time.sleep(0.1)
    request = SetGoalPose.Request()
    request.goal_pose.header.frame_id = 'map'
    request.goal_pose.pose.position.x = -1.0
    request.goal_pose.pose.orientation.w = 1.0
    future = client.call_async(request)
    assert wait_until(future.done, harness)
    assert future.result().success

    expected = [
        'pause:true', 'clear_route', 'local', 'enable', 'engage', 'pause:false']
    assert wait_until(lambda: harness.events[:6] == expected, harness)


def test_completed_reverse_automatically_returns_to_autonomous(running_system):
    harness = running_system
    client = harness.node.create_client(
        SetGoalPose, f'/{NODE_NAME}/set_goal_pose')
    assert client.wait_for_service(timeout_sec=5.0)
    assert wait_until(lambda: harness.odom_pub.get_subscription_count() > 0, harness)

    request = SetGoalPose.Request()
    request.goal_pose.header.frame_id = 'map'
    request.goal_pose.pose.position.x = -1.0
    request.goal_pose.pose.orientation.w = 1.0
    harness.publish_odometry()
    future = client.call_async(request)
    assert wait_until(future.done, harness)
    assert future.result().success
    assert wait_until(lambda: 'pause:false' in harness.events, harness)

    expected = [
        'pause:true', 'clear_route', 'local', 'enable', 'engage', 'pause:false',
        'pause:true', 'autonomous', 'pause:false']
    assert wait_until(
        lambda: harness.events[:9] == expected,
        harness,
        timeout=5.0,
        odom_x=-1.0)


def test_transient_mode_transition_is_retried(running_system):
    harness = running_system
    harness.local_failures_remaining = 1
    client = harness.node.create_client(
        SetGoalPose, f'/{NODE_NAME}/set_goal_pose')
    assert client.wait_for_service(timeout_sec=5.0)
    assert wait_until(lambda: harness.odom_pub.get_subscription_count() > 0, harness)

    request = SetGoalPose.Request()
    request.goal_pose.header.frame_id = 'map'
    request.goal_pose.pose.position.x = -1.0
    request.goal_pose.pose.orientation.w = 1.0
    harness.publish_odometry()
    future = client.call_async(request)
    assert wait_until(future.done, harness)
    assert future.result().success

    expected = [
        'pause:true', 'clear_route', 'local:transition', 'local',
        'enable', 'engage', 'pause:false']
    assert wait_until(lambda: harness.events[:7] == expected, harness)
