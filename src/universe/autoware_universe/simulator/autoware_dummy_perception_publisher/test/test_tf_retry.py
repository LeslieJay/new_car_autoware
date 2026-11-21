# Copyright 2026 TIER IV, Inc.
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
"""Exercise the actual publisher with TF arriving after its lookup timeout."""
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from autoware_perception_msgs.msg import TrackedObjects
from tier4_simulation_msgs.msg import DummyObject


@pytest.mark.parametrize('tf_delay,expect_output', [(0.65, True), (2.0, False)])
def test_delayed_tf_preserves_sample(tmp_path, tf_delay, expect_output):
    executable = Path(get_package_prefix('autoware_dummy_perception_publisher')) / (
        'lib/autoware_dummy_perception_publisher/autoware_dummy_perception_publisher_node')
    rclpy.init()
    node = rclpy.create_node('tf_retry_driver')
    broadcaster = TransformBroadcaster(node)
    samples = []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(
        TrackedObjects,
        '/tf_retry/dummy_perception_publisher/output/debug/ground_truth_objects',
        lambda msg: samples.append(msg) if msg.objects else None, qos)
    publisher = node.create_publisher(DummyObject, '/tf_retry/input/object', 10)

    def publish_tf():
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.header.stamp = (node.get_clock().now() - Duration(seconds=tf_delay)).to_msg()
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

    timer = node.create_timer(0.02, publish_tf)
    log_path = tmp_path / 'publisher.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '--params-file',
            str(executable.parents[2] / 'share/autoware_dummy_perception_publisher/config/dummy_perception_publisher.param.yaml'),
            '-r', '__ns:=/tf_retry',
            '-p', 'publish_ground_truth:=true', '-p', 'use_base_link_z:=false',
            '-p', 'object_centric_pointcloud:=true', '-p', 'enable_ray_tracing:=false',
            '-p', 'visible_range:=30.0'], stdout=log, stderr=subprocess.STDOUT,
            env=os.environ.copy())
        try:
            until = time.monotonic() + 5.0
            while publisher.get_subscription_count() == 0 and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert publisher.get_subscription_count() > 0
            obj = DummyObject()
            obj.header.frame_id = 'map'
            obj.header.stamp = node.get_clock().now().to_msg()
            origin = obj.header.stamp.sec + obj.header.stamp.nanosec / 1e9
            obj.id.uuid[0] = 1
            obj.action = DummyObject.ADD
            obj.classification.label = 1
            obj.classification.probability = 1.0
            obj.shape.type = 0
            obj.shape.dimensions.x = obj.shape.dimensions.y = obj.shape.dimensions.z = 1.0
            obj.initial_state.pose_covariance.pose.position.x = 5.0
            obj.initial_state.pose_covariance.pose.orientation.w = 1.0
            obj.initial_state.twist_covariance.twist.linear.x = 0.3
            obj.min_velocity, obj.max_velocity = -10.0, 10.0
            publisher.publish(obj)
            until = time.monotonic() + 8.0
            while time.monotonic() < until and (not expect_output or len(samples) < 3):
                rclpy.spin_once(node, timeout_sec=0.02)
            assert process.poll() is None, log_path.read_text()
            if expect_output:
                assert len(samples) >= 3, log_path.read_text()[-3000:]
                stamps = [m.header.stamp.sec + m.header.stamp.nanosec / 1e9 for m in samples]
                assert stamps == sorted(set(stamps))
                for msg, stamp in zip(samples, stamps):
                    x = msg.objects[0].kinematics.pose_with_covariance.pose.position.x
                    assert abs(x - (5.0 + 0.3 * (stamp - origin))) < 0.03
            else:
                assert not samples, 'Expired TF samples must not be published'
        finally:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_timer(timer)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
