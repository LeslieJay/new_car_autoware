#!/usr/bin/env python3
"""Closed-loop admission cases for avoidance-by-lane-change on hy_1031."""
import argparse
import json
import math
import re
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import RouteOption
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from geometry_msgs.msg import Pose
from tier4_simulation_msgs.msg import DummyObject
from tier4_rtc_msgs.msg import CooperateStatusArray
from autoware_map_msgs.msg import LaneletMapBin
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from campus_agv_scenario import CampusScenario, polygon, polygon_distance, recorded_validation
from dummy_object_utils import make_dummy_object, yaw_to_quaternion
from simple_avoidance_distance_test import (
    DistanceNode, load_yaml, point_at_reference_s, project_to_reference)


CASES = {
    'turn_outside', 'outside_right_005', 'outside_left_030', 'tangent_right',
    'adjacent_lane', 'past_path_end', 'move_in_out', 'move_deep_in_out', 'inside_right_030',
    'inside_left_050', 'inside_center',
}
OUTSIDE = {'turn_outside', 'outside_right_005', 'outside_left_030',
           'tangent_right', 'adjacent_lane', 'past_path_end'}


class AdmissionScenario(CampusScenario):
    def __init__(self, cfg, params, args):
        self.candidate_samples = 0
        self.module_stop_samples = 0
        self.module_factor_samples = 0
        self.rtc_request_samples = 0
        self.unexplained_stop_max = 0.0
        self.stop_since = None
        self.first_object_seen_at = None
        self.arrival_at = None
        self.move_stage = 0
        self.move_started_at = None
        self.candidate_during_intrusion = False
        self.candidate_after_exit = False
        self.admission_timeline = []
        self.map_msg = None
        self.map_relay_count = 0
        super().__init__(cfg, params, args)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(LaneletMapBin, '/map/vector_map', map_qos)
        self.create_subscription(LaneletMapBin, '/map/vector_map', self._map, map_qos)
        self.create_timer(1.0, self._relay_map)
        for side in ('left', 'right'):
            self.create_subscription(CooperateStatusArray,
                f'/planning/cooperate_status/avoidance_by_lane_change_{side}',
                self._rtc_status, 10)

    def _call_service(self, client, request, wait_sec=10.0, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=wait_sec):
            self.admission_timeline.append([time.time(), 'service_unavailable', client.srv_name])
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        status = getattr(response, 'status', None)
        success = bool(response is not None and
                       (getattr(status, 'success', True) if status is not None else
                        getattr(response, 'success', True)))
        self.admission_timeline.append([time.time(), 'service_result',
            {'name': client.srv_name, 'success': success,
             'code': getattr(status, 'code', None),
             'message': getattr(status, 'message', '')}])
        return success

    def publish_setup(self, repeat=3, engage=True):
        super().publish_setup(repeat, engage)
        if not self.setup_pose_ready:
            return
        if self.route_state == RouteState.SET:
            return
        goal = self.cfg['goal']
        request = SetRoutePoints.Request()
        request.header.frame_id = self.cfg['frame_id']
        request.option = RouteOption(allow_goal_modification=False)
        request.goal = Pose()
        request.goal.position.x = float(goal['x'])
        request.goal.position.y = float(goal['y'])
        request.goal.position.z = float(goal.get('z', 0.0))
        request.goal.orientation = yaw_to_quaternion(float(goal['yaw']))
        for _ in range(5):
            request.header.stamp = self.get_clock().now().to_msg()
            self._call_service(self.route_service, request, wait_sec=5.0, timeout_sec=8.0)
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.route_state == RouteState.SET:
                self.set_autonomous(engage=True)
                return
        self.admission_timeline.append([time.time(), 'route_not_set', self.route_state])

    def _map(self, msg):
        if self.map_msg is None:
            self.map_msg = msg
            self.admission_timeline.append([time.time(), 'map_received', len(msg.data)])

    def _relay_map(self):
        # The map loader publishes once. On this simulator Fast DDS can miss
        # its large transient message while motion planning is still loading.
        if (self.armed and self.map_msg is not None and not self.latest_trajectory
                and self.map_relay_count < 10):
            self.map_pub.publish(self.map_msg)
            self.map_relay_count += 1
            self.admission_timeline.append([time.time(), 'map_republished',
                                            self.map_relay_count])

    def _rtc_status(self, msg):
        if self.a_seen and msg.statuses:
            self.rtc_request_samples += len(msg.statuses)
            self.admission_timeline.append([time.time(), 'rtc_status',
                                            [int(s.state.type) for s in msg.statuses]])

    def _objects(self, msg):
        super()._objects(msg)
        if self.a_seen and self.first_object_seen_at is None:
            self.first_object_seen_at = time.monotonic()
            self.admission_timeline.append([time.time(), 'object_received', 1])

    def _route_state(self, msg):
        super()._route_state(msg)
        if msg.state == RouteState.ARRIVED and self.arrival_at is None:
            self.arrival_at = time.monotonic()

    def _candidate(self, msg):
        super()._candidate(msg)
        if self.a_seen and msg.points:
            self.candidate_samples += 1
            self.admission_timeline.append([time.time(), 'candidate', len(msg.points)])
            if self.args.scenario in ('move_in_out', 'move_deep_in_out'):
                if self.move_stage == 1:
                    self.candidate_during_intrusion = True
                if (self.move_stage >= 2 and self.move_started_at is not None and
                        time.monotonic() - self.move_started_at > 1.0):
                    self.candidate_after_exit = True

    def _simple_factors(self, msg):
        super()._simple_factors(msg)
        if self.a_seen and msg.factors:
            self.module_factor_samples += 1
            self.module_stop_samples += any(int(f.behavior) == 3 for f in msg.factors)
            self.admission_timeline.append([time.time(), 'module_factors',
                                            [int(f.behavior) for f in msg.factors]])

    def _odom(self, msg):
        if self.args.scenario != 'turn_outside':
            super()._odom(msg)
        else:
            DistanceNode._odom(self, msg)
            p = msg.pose.pose.position
            self.moving |= self.armed and abs(msg.twist.twist.linear.x) > 0.05
            if self.moving and self.motion_start_stamp is None:
                self.motion_start_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            body = polygon(p.x, p.y, yaw, -0.676, 1.554, 0.6525)
            obj = self.obstacle_msg
            if obj is not None:
                op = obj.initial_state.pose_covariance.pose.position
                obstacle = polygon(op.x, op.y, -1.6592073,
                                   -obj.shape.dimensions.x/2, obj.shape.dimensions.x/2,
                                   obj.shape.dimensions.y/2)
                self.min_clearance['A'] = min(self.min_clearance['A'],
                                              polygon_distance(body, obstacle))
        goal = self.cfg['goal']
        before_goal = math.hypot(self.latest_pose[0] - goal['x'],
                                 self.latest_pose[1] - goal['y']) > 2.0
        if self.moving and self.route_state != RouteState.ARRIVED and before_goal:
            now = time.monotonic()
            if abs(self.latest_speed) < 0.05:
                if self.stop_since is None:
                    self.stop_since = now
                self.unexplained_stop_max = max(self.unexplained_stop_max,
                                                now - self.stop_since)
            else:
                self.stop_since = None
        else:
            self.stop_since = None

    def publish_obstacle(self, longitudinal, intrusion, shoulder, anchor_xy=None, visible=True):
        case = self.args.scenario
        if case == 'turn_outside':
            x, y, yaw = -31.113, -38.350, -1.6592073
        else:
            anchor = anchor_xy or (self.ego['x'], self.ego['y'])
            anchor_s, _ = project_to_reference(*anchor, self.baseline_path)
            location = point_at_reference_s(self.baseline_path, anchor_s + longitudinal)
            x = location[0] if location else anchor[0] - longitudinal
            yaw = math.pi
            upper = 6.0884 + (x + 45.0319) * (6.2051 - 6.0884) / (-323.0052 + 45.0319)
            lower = 1.5884 + (x + 45.0338) * (1.7051 - 1.5884) / (-323.0071 + 45.0338)
            half = 0.75
            if case == 'inside_center':
                y = (upper + lower) / 2
            elif shoulder == 'right':
                y = upper - intrusion + half
            else:
                y = lower + intrusion - half
            if case == 'past_path_end':
                x = -140.0
            if case in ('move_in_out', 'move_deep_in_out') and self.first_object_seen_at is not None:
                elapsed = time.monotonic() - self.first_object_seen_at
                exit_after = 12.0 if case == 'move_deep_in_out' else 6.0
                desired = 1 if 3.0 <= elapsed < exit_after else 2 if elapsed >= exit_after else 0
                if desired != self.move_stage:
                    self.move_stage = desired
                    self.move_started_at = time.monotonic()
                if desired == 1:
                    y -= 1.75 if case == 'move_deep_in_out' else 0.35
        if self.obstacle_msg is None:
            self.obstacle_msg = make_dummy_object(
                'map', (self.get_clock().now() - Duration(seconds=0.3)).to_msg(),
                x, y, 0.0, yaw,
                length=4.209 if case == 'turn_outside' else 2.0,
                width=1.835 if case == 'turn_outside' else 1.5, height=1.5)
            self.obstacle_msg.initial_state.pose_covariance.covariance = [0.0] * 36
        obj = self.obstacle_msg
        obj.header.stamp = (self.get_clock().now() - Duration(seconds=0.3)).to_msg()
        obj.initial_state.pose_covariance.pose.position.x = x
        obj.initial_state.pose_covariance.pose.position.y = y
        obj.action = DummyObject.MODIFY if self.a_seen else DummyObject.ADD
        self.object_pub.publish(obj)
        self.armed = True
        return x, y, yaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=sorted(CASES), required=True)
    parser.add_argument('--launch-log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    cfg = load_yaml(root/'config/simple_avoidance_distance.yaml')
    if args.scenario == 'turn_outside':
        cfg['ego'] = dict(x=-30.5, y=-1.2, z=0.0, yaw=-0.33)
        cfg['goal'] = dict(x=-100.0, y=3.86, z=0.0, yaw=math.pi)
    elif args.scenario == 'past_path_end':
        cfg['ego'] = dict(x=-100.0, y=3.86, z=0.0, yaw=math.pi)
        cfg['goal'] = dict(x=-130.0, y=3.87, z=0.0, yaw=math.pi)
    else:
        cfg['ego'] = dict(x=-100.0, y=3.86, z=0.0, yaw=math.pi)
        cfg['goal'] = dict(x=-220.0, y=3.912, z=0.0, yaw=math.pi)
    cfg['obstacle']['lane_yaw'] = math.pi
    cfg['obstacle']['lane_width'] = 4.5
    cfg['test'].update(sample_sec=180.0 if args.scenario in OUTSIDE else 270.0,
                       settle_sec=60.0, obstacle_hold_sec=400.0,
                       require_route_arrival=True)
    placement = {
        'outside_right_005': (42.0, -0.05, 'right'),
        'outside_left_030': (42.0, -0.30, 'left'),
        'tangent_right': (42.0, 0.0, 'right'),
        'adjacent_lane': (42.0, -0.30, 'left'),
        'past_path_end': (40.0, 0.30, 'right'),
        'move_in_out': (42.0, -0.05, 'right'),
        'move_deep_in_out': (42.0, -0.05, 'right'),
        'inside_right_030': (42.0, 0.30, 'right'),
        'inside_left_050': (42.0, 0.50, 'left'),
        'inside_center': (42.0, 2.25, 'right'),
        'turn_outside': (42.0, 0.0, 'right'),
    }
    params = load_yaml(Path('/home/nvidia/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/campus_agv.param.yaml'))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir/'effective_input.json').write_text(json.dumps(
        {'config': cfg, 'params': params, 'placement': placement[args.scenario]}, indent=2))
    rclpy.init()
    node = AdmissionScenario(cfg, params, args)
    try:
        row = node.run_case(1.5, *placement[args.scenario], args.output_dir/'bag')
        node.read_diagnostics()
        recorded = recorded_validation(args.output_dir/'bag', node.motion_start_stamp)
        log = args.launch_log.read_text(errors='replace')
        borrowing = re.findall(r'\[OPPOSITE_BORROWING\] phase=([^\s]+)', log)
        arrived = node.route_state == RouteState.ARRIVED
        clear = node.min_clearance['A']
        result = dict(scenario=args.scenario, object_received=node.a_seen,
                      arrived=arrived, moving=node.moving,
                      candidate_samples=node.candidate_samples,
                      module_factor_samples=node.module_factor_samples,
                      module_stop_samples=node.module_stop_samples,
                      rtc_request_samples=node.rtc_request_samples,
                      borrowing_phases=borrowing, max_stop_sec=node.unexplained_stop_max,
                      clearance_m=clear if math.isfinite(clear) else None,
                      boundary_violations=node.boundary_violations,
                      validation=recorded, final_pose=node.latest_pose,
                      driver_result=row.get('result'),
                      driver_failure_reasons=row.get('failure_reasons'),
                      trajectory_samples=len(node.trajectory_history),
                      candidate_during_intrusion=node.candidate_during_intrusion,
                      candidate_after_exit=node.candidate_after_exit,
                      object_to_arrival_sec=(node.arrival_at - node.first_object_seen_at
                          if node.arrival_at is not None and node.first_object_seen_at is not None
                          else None),
                      bag_returncode=row.get('bag_returncode'),
                      map_received=node.map_msg is not None,
                      map_relay_count=node.map_relay_count,
                      route_state=node.route_state,
                      bag_present=bool(list((args.output_dir/'bag').glob('*.db3'))))
        limit = 180.0 if args.scenario in OUTSIDE else 270.0
        common = (node.map_msg is not None and node.a_seen and node.moving and result['bag_present'] and
                  result['object_to_arrival_sec'] is not None and
                  result['object_to_arrival_sec'] <= limit and
                  row.get('bag_returncode') == 0 and recorded['samples'] > 0)
        if args.scenario in OUTSIDE:
            result['passed'] = (common and arrived and node.candidate_samples == 0 and
                node.rtc_request_samples == 0 and node.module_stop_samples == 0 and not borrowing and
                node.unexplained_stop_max <= 5.0)
        elif args.scenario in ('move_in_out', 'move_deep_in_out'):
            result['passed'] = (common and arrived and node.candidate_during_intrusion and
                not node.candidate_after_exit and node.module_stop_samples == 0)
        else:
            result['passed'] = (common and arrived and node.complete_seen and
                clear > 0 and node.boundary_violations == 0 and
                recorded['non_latency_invalid_count'] == 0)
        (args.output_dir/'verdict.json').write_text(json.dumps(result, indent=2))
        (args.output_dir/'state_timeline.json').write_text(json.dumps(
            node.admission_timeline + [[sample[0], 'speed_mps', sample[1]]
                                       for sample in node.speed_history], indent=2))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(2, 1, figsize=(11, 7))
        if node.actual:
            axes[0].plot([p[0] for p in node.actual], [p[1] for p in node.actual])
            axes[1].plot([t - node.actual_time_history[0] for t in node.actual_time_history],
                         [p[2] for p in node.actual])
        obj = node.obstacle_msg
        if obj is not None:
            p = obj.initial_state.pose_covariance.pose.position
            axes[0].scatter([p.x], [p.y], c='red', marker='x')
        axes[0].set(xlabel='map x [m]', ylabel='map y [m]', aspect='equal')
        axes[1].set(xlabel='elapsed time [s]', ylabel='actual speed [m/s]')
        figure.suptitle(args.scenario)
        figure.tight_layout()
        figure.savefig(args.output_dir/'actual_trajectory.png', dpi=150)
        plt.close(figure)
        print(json.dumps(result), flush=True)
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
