#!/usr/bin/env python3
"""One closed-loop campus scenario. Run against a fresh planning_simulator instance."""
import argparse
import json
import math
import re
import sqlite3
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from autoware_vehicle_msgs.msg import Engage as VehicleEngage, GearCommand
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from autoware_planning_validator.msg import PlanningValidatorStatus
from tier4_simulation_msgs.msg import DummyObject
from autoware_adapi_v1_msgs.msg import RouteState, RouteOption
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from geometry_msgs.msg import Pose
from simple_avoidance_distance_test import DistanceNode, load_yaml
from dummy_object_utils import make_dummy_object, yaw_to_quaternion
from opposite_borrowing_metrics import observe


class CampusScenario(DistanceNode):
    def __init__(self, cfg, params, args):
        self.args = args
        self.armed = False
        self.b = None
        self.b_seen = False
        self.a_seen = False
        self.invalid_count = 0
        self.validation_samples = 0
        self.moving = False
        self.motion_start_stamp = None
        self.log_offset = 0
        self.phase = ''
        self.return_x = None
        self.extension_conflict_id = None
        self.original_return_x = None
        self.extension = 0.0
        self.complete_seen = False
        self.min_clearance = {'A': float('inf'), 'B': float('inf')}
        self.boundary_violations = 0
        self.trigger_pose = None
        self.stopped_since = None
        self.b_spawn_time = None
        self.b_origin_x = None
        self.b_stop_time = None
        self.b_wait_since = None
        self.waited_for_b = False
        self.first_b_extension_time = None
        self.early_b_extension = False
        self.timeline = []
        self.extension_events = []
        self.clearance_samples = {'A': 0, 'B': 0}
        self.b_perception_samples = 0
        self.b_perception_ids = set()
        self.observation_time = params['/**']['ros__parameters']['avoidance_by_lane_change']['opposite_lane']['static_observation_time']
        super().__init__(cfg, 'avoidance_by_lane_change', params, 'start_lock', args.launch_log,
                         'normal', 1.5, [], case_name=args.scenario)
        self.engage_pub = self.create_publisher(VehicleEngage, '/vehicle/engage', 10)
        self.direct_gear_pub = self.create_publisher(GearCommand, '/control/command/gear_cmd',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.1, self.keep_motion_enabled)
        self.create_subscription(PlanningValidatorStatus, '/planning/planning_validator/validation_status', self.validation, 10)

    def _call_service(self, client, request, wait_sec=10.0, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=wait_sec):
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        status = getattr(response, 'status', None)
        success = bool(response is not None and
                       (getattr(status, 'success', True) if status is not None else
                        getattr(response, 'success', True)))
        self.timeline.append(dict(event='service_result', stamp=time.time(),
                                  service=client.srv_name, success=success,
                                  code=getattr(status, 'code', None),
                                  message=getattr(status, 'message', '')))
        return success

    def publish_setup(self, repeat=3, engage=True):
        super().publish_setup(repeat, engage)
        if not self.setup_pose_ready:
            return
        # Match the existing lane-admission harness: an API response can precede
        # mission-planner readiness. Never reset the vehicle pose after route hand-off.
        if self.route_state != RouteState.SET:
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
                self._call_service(self.route_service, request, wait_sec=5.0, timeout_sec=5.0)
                for _ in range(20):
                    rclpy.spin_once(self, timeout_sec=0.1)
                if self.route_state == RouteState.SET:
                    break
        # Use the runner's existing startup budget for actual inputs, not only node loading.
        # This is before obstacle injection and the unchanged 300 s scenario window.
        deadline = time.monotonic() + 150.0
        while (rclpy.ok() and time.monotonic() < deadline and
               (self.route_state != RouteState.SET or not self.latest_behavior or
                not self.latest_trajectory)):
            rclpy.spin_once(self, timeout_sec=0.05)
            self.publish_speed_limit()
            self.publish_trailer_configuration()
            self.publish_drive_gear()
        ready = self.route_state == RouteState.SET and bool(self.latest_behavior and self.latest_trajectory)
        self.timeline.append(dict(event='setup_inputs_ready', stamp=time.time(), ready=ready,
                                  route_state=self.route_state))

    def metrics(self, speed, longitudinal, intrusion, shoulder, ox, oy):
        # Campus acceptance uses measured body geometry and the recorded validator stream.
        # Avoid the generic driver's repeated projections of every historical candidate point.
        return dict(result="RECORDED", failure_reasons=self.invalid_reason or "",
                    object_seen=self.a_seen, obstacle_x=ox, obstacle_y=oy,
                    speed_mps=speed, longitudinal_m=longitudinal)

    def scenario_complete(self):
        if self.args.scenario == 'blocked' and self.b_seen and self.moving and abs(self.latest_speed) < 0.05:
            if self.stopped_since is None:
                self.stopped_since = time.monotonic()
            return time.monotonic() - self.stopped_since >= 3.0
        self.stopped_since = None
        return False

    def keep_motion_enabled(self):
        if self.armed:
            engage = VehicleEngage()
            engage.stamp = self.get_clock().now().to_msg()
            engage.engage = True
            self.engage_pub.publish(engage)
            gear = GearCommand()
            gear.stamp = engage.stamp
            gear.command = GearCommand.DRIVE
            self.direct_gear_pub.publish(gear)
            self.publish_drive_gear()
            self.publish_speed_limit()
            self.publish_trailer_configuration()

    def validation(self, msg):
        if self.moving:
            self.validation_samples += 1
        if self.moving and any(name.startswith('is_valid') and getattr(msg, name) is False
                               for name in msg.get_fields_and_field_types()):
            self.invalid_count += 1

    def read_diagnostics(self):
        if not self.args.launch_log.exists():
            return
        with self.args.launch_log.open() as stream:
            stream.seek(self.log_offset)
            lines = stream.readlines()
            self.log_offset = stream.tell()
        for line in lines:
            if '[OPPOSITE_BORROWING]' not in line or 'phase=' not in line:
                continue
            observe(self.borrowing_metrics, line)
            fields = dict(re.findall(r'([a-z_]+)=([^\s\x1b]+)', line))
            self.phase = fields.get('phase', self.phase)
            self.complete_seen |= self.phase == 'complete'
            if (self.original_return_x is None and fields.get('safe') == '1' and
                    fields.get('phase') in ['borrowing', 'passing', 'returning'] and 'return_x' in fields):
                self.original_return_x = float(fields['return_x'])
            if 'return_x' in fields:
                self.return_x = float(fields['return_x'])
            if fields.get('return_extension_conflict', 'none') != 'none':
                self.extension_conflict_id = fields['return_extension_conflict']
            if 'planned_return_s' in fields:
                self.extension = max(self.extension, float(fields['planned_return_s']) - float(fields['original_return_s']))
            conflict = fields.get('return_extension_conflict', 'none')
            if conflict != 'none':
                stamps = re.findall(r'\[(\d{9,}\.\d+)\]', line)
                self.extension_events.append((conflict, float(stamps[0]) if stamps else None))

    def _objects(self, msg):
        super()._objects(msg)
        if self.obstacle_msg is not None:
            target = self.obstacle_msg.initial_state.pose_covariance.pose.position
            self.a_seen |= any(math.hypot(o.kinematics.initial_pose_with_covariance.pose.position.x-target.x,
                                         o.kinematics.initial_pose_with_covariance.pose.position.y-target.y) < 2.0
                               for o in msg.objects)
        if self.b:
            target = self.b.initial_state.pose_covariance.pose.position
            matches = [o for o in msg.objects
                       if math.hypot(o.kinematics.initial_pose_with_covariance.pose.position.x-target.x,
                                     o.kinematics.initial_pose_with_covariance.pose.position.y-target.y) < 2.0]
            self.b_perception_ids.update(bytes(o.object_id.uuid).hex() for o in matches)
            seen = bool(matches)
            self.b_perception_samples += int(seen)
            self.b_seen |= seen

    def _odom(self, msg):
        super()._odom(msg)
        p = msg.pose.pose.position
        self.moving |= self.armed and abs(msg.twist.twist.linear.x) > 0.05
        if self.moving and self.motion_start_stamp is None:
            self.motion_start_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        body = polygon(p.x, p.y, yaw, -0.676, 1.554, 0.6525)
        if self.moving:
            # Outer boundaries of the straight 906/907 pair, interpolated from map nodes.
            for x, y in body:
                bottom = -2.9116 + (x+45.0357)*(-2.7949+2.9116)/(-323.009+45.0357)
                top = 6.0884 + (x+45.0319)*(6.2051-6.0884)/(-323.0052+45.0319)
                if not (bottom <= y <= top):
                    self.boundary_violations += 1
                    break
        for name, obj in [('A', self.obstacle_msg), ('B', self.b)]:
            if obj is None:
                continue
            op = obj.initial_state.pose_covariance.pose.position
            oq = obj.initial_state.pose_covariance.pose.orientation
            angle = math.atan2(2*oq.w*oq.z, 1-2*oq.z*oq.z)
            actual_y = op.y
            if self.args.scenario == 'pedestrian' and name == 'B' and self.b_spawn_time is not None:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                actual_y += max(0.0, stamp - self.b_spawn_time)
            actual_x = op.x
            if self.args.scenario == 'moving_then_static' and name == 'B':
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                actual_x = self.b_origin_x - 0.3 * max(0.0, min(stamp, self.b_stop_time or stamp) - self.b_spawn_time)
                if self.b_stop_time is None:
                    if self.b_seen and abs(msg.twist.twist.linear.x) < 0.05:
                        if self.b_wait_since is None:
                            self.b_wait_since = stamp
                        if stamp - self.b_wait_since >= 2.0:
                            self.waited_for_b = True
                    else:
                        self.b_wait_since = None
            obstacle = polygon(actual_x, actual_y, angle, -obj.shape.dimensions.x/2,
                               obj.shape.dimensions.x/2, obj.shape.dimensions.y/2)
            self.clearance_samples[name] += 1
            self.min_clearance[name] = min(self.min_clearance[name], polygon_distance(body, obstacle))

    def publish_obstacle(self, *args, **kwargs):
        result = super().publish_obstacle(*args, **kwargs)
        self.armed = True
        self.read_diagnostics()
        scenario = self.args.scenario
        x, y = self.latest_pose
        passing = self.phase == 'passing' and y < 1.0
        returning = self.phase == 'returning' and y > -0.5 and y < 2.5
        trigger = passing if scenario != 'returning' else returning
        if scenario != 'baseline' and self.b is None and trigger and self.original_return_x is not None:
            bx = result[0]-7.0 if scenario == 'gap5' else self.original_return_x+8.0
            if scenario == 'returning':
                bx = self.original_return_x + 2.0
            if scenario == 'pedestrian':
                bx = x-8.0
            self.b = make_dummy_object('map', self.get_clock().now().to_msg(), bx,
                                      -6.0 if scenario == 'pedestrian' else 3.9, 0.0,
                                      math.pi/2 if scenario == 'pedestrian' else math.pi,
                                      length=0.6 if scenario == 'pedestrian' else 2.0,
                                      width=20.0 if scenario == 'blocked' else 0.6 if scenario == 'pedestrian' else 1.5,
                                      height=1.7 if scenario == 'pedestrian' else 1.5, velocity=1.0 if scenario == 'pedestrian' else 0.3 if scenario == 'moving_then_static' else 0.0)
            if scenario == 'pedestrian':
                self.b.classification.label = 7
            elif scenario == 'blocked':
                self.b.classification.label = 0
            self.trigger_pose = [x, y]
            self.b_spawn_time = self.get_clock().now().nanoseconds / 1e9
            self.b_origin_x = bx
            self.timeline.append(dict(event='b_spawn', stamp=self.b_spawn_time))
            self.b.action = DummyObject.ADD
            self.object_pub.publish(self.b)
        elif self.b is not None:
            if scenario != 'pedestrian':
                if scenario == 'moving_then_static':
                    now = self.get_clock().now().nanoseconds / 1e9
                    if self.waited_for_b and self.b_stop_time is None:
                        self.b_stop_time = now
                        self.timeline.append(dict(event='b_stop_after_ego_wait', stamp=now))
                    self.b.initial_state.pose_covariance.pose.position.x = self.b_origin_x - 0.3 * max(0.0, (self.b_stop_time or now) - self.b_spawn_time)
                    self.b.initial_state.twist_covariance.twist.linear.x = 0.0 if self.b_stop_time else 0.3
                self.b.header.stamp = self.get_clock().now().to_msg()
                self.b.action = DummyObject.MODIFY
                self.object_pub.publish(self.b)
        return result


def polygon(x, y, yaw, lo, hi, half):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+u*c-v*s, y+u*s+v*c) for u,v in [(lo,-half),(hi,-half),(hi,half),(lo,half)]]


def polygon_distance(a, b):
    separated = False
    for poly in (a,b):
        for i in range(4):
            dx = poly[(i+1)%4][0]-poly[i][0]
            dy = poly[(i+1)%4][1]-poly[i][1]
            pa = [-dy*x+dx*y for x,y in a]
            pb = [-dy*x+dx*y for x,y in b]
            separated |= max(pa) < min(pb) or max(pb) < min(pa)
    if not separated:
        return 0.0
    def distance(p, u, v):
        dx,dy = v[0]-u[0],v[1]-u[1]
        t = max(0.0,min(1.0,((p[0]-u[0])*dx+(p[1]-u[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-u[0]-t*dx,p[1]-u[1]-t*dy)
    return min([distance(p,b[i],b[(i+1)%4]) for p in a for i in range(4)]+
               [distance(p,a[i],a[(i+1)%4]) for p in b for i in range(4)])


def recorded_validation(bag_dir, start_stamp):
    """The recorded stream is authoritative; a busy live subscriber may miss samples."""
    from rclpy.serialization import deserialize_message
    summary = {'samples': 0, 'invalid_count': 0, 'non_latency_invalid_count': 0, 'invalid_fields': {}}
    for database in sorted(bag_dir.glob('*.db3')):
        with sqlite3.connect(database) as connection:
            topic = connection.execute("SELECT id FROM topics WHERE name=?",
                ('/planning/planning_validator/validation_status',)).fetchone()
            if not topic:
                continue
            for timestamp, payload in connection.execute(
                    'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', topic):
                if start_stamp is None or timestamp / 1e9 < start_stamp:
                    continue
                message = deserialize_message(payload, PlanningValidatorStatus)
                summary['samples'] += 1
                invalid = [name for name in message.get_fields_and_field_types()
                           if name.startswith('is_valid') and getattr(message, name) is False]
                summary['invalid_count'] += bool(invalid)
                summary['non_latency_invalid_count'] += any(field != 'is_valid_latency' for field in invalid)
                for field in invalid:
                    summary['invalid_fields'][field] = summary['invalid_fields'].get(field, 0) + 1
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['baseline','gap5','passing','returning','blocked','pedestrian','moving_then_static'], required=True)
    parser.add_argument('--launch-log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--sample-sec', type=float, default=300.0)
    parser.add_argument('--goal-x', type=float, default=-220.0)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    cfg = load_yaml(root/'config/simple_avoidance_distance.yaml')
    cfg['ego'] = {'x':-100.0,'y':3.86,'z':0.0,'yaw':math.pi}
    cfg['goal'] = {'x':args.goal_x,'y':3.912,'z':0.0,'yaw':math.pi}
    cfg['obstacle']['lane_yaw'] = math.pi
    cfg['test'].update(sample_sec=args.sample_sec, obstacle_hold_sec=args.sample_sec+120,
                       require_route_arrival=True)
    launch_config = (Path(get_package_share_directory('autoware_launch')) /
                     'config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner')
    params = load_yaml(launch_config/'avoidance_by_lane_change/campus_agv.param.yaml')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    expansion = load_yaml(launch_config/'campus_agv_drivable_area_expansion.param.yaml')
    (args.output_dir/'effective_input.json').write_text(json.dumps({'config':cfg,'params':params,'drivable_area_params':expansion},indent=2))
    rclpy.init()
    node = CampusScenario(cfg, params, args)
    try:
        row = node.run_case(1.5,42.0,1.0,'right',args.output_dir/'bag')
        node.read_diagnostics()
        extensions = [stamp for conflict, stamp in node.extension_events
                      if conflict in node.b_perception_ids]
        if extensions:
            node.early_b_extension = any(stamp is None for stamp in extensions)
            valid_stamps = [stamp for stamp in extensions if stamp is not None]
            if valid_stamps:
                node.first_b_extension_time = min(valid_stamps)
                confirmed_since = node.b_stop_time if args.scenario == 'moving_then_static' else node.b_spawn_time
                node.early_b_extension |= confirmed_since is None or node.first_b_extension_time < confirmed_since + node.observation_time
                node.timeline.append(dict(event='first_b_extension', stamp=node.first_b_extension_time))
        recorded = recorded_validation(args.output_dir/'bag', node.motion_start_stamp)
        node.invalid_count = recorded['non_latency_invalid_count']
        node.validation_samples = recorded['samples']
        arrived = node.route_state == RouteState.ARRIVED
        clearances = {k:v if math.isfinite(v) else None for k,v in node.min_clearance.items()}
        (args.output_dir/'legacy_driver_metrics.json').write_text(json.dumps(row, indent=2))
        obstacles = {}
        for name, obj in [('A', node.obstacle_msg), ('B', node.b)]:
            if obj is not None:
                position = obj.initial_state.pose_covariance.pose.position
                obstacles[name] = dict(id=bytes(obj.id.uuid).hex(), x=position.x, y=position.y,
                    length=obj.shape.dimensions.x, width=obj.shape.dimensions.y)
        result = dict(scenario=args.scenario, arrived=arrived, complete=node.complete_seen,
                      a_seen=bool(row.get('object_seen')), b_seen=node.b_seen, trigger_pose=node.trigger_pose,
                      obstacles=obstacles, original_return_x=node.original_return_x,
                      accepted_return_x=node.return_x, extension_conflict_id=node.extension_conflict_id,
                      extension_m=node.extension, invalid_trajectory_count=node.invalid_count,
                      validation_samples=node.validation_samples, recorded_validation=recorded,
                      acceptance_policy="safe_avoidance_and_arrival_latency_not_gated",
                      min_body_clearance_m=clearances, boundary_violations=node.boundary_violations,
                      final_pose=node.latest_pose, final_speed=node.latest_speed, route_state=node.route_state,
                      first_b_extension_time=node.first_b_extension_time,
                      early_b_extension=node.early_b_extension, waited_for_b=node.waited_for_b,
                      clearance_samples=node.clearance_samples, timeline=node.timeline,
                      b_perception_samples=node.b_perception_samples,
                      b_perception_ids=sorted(node.b_perception_ids))
        result['passed'] = (bool(row.get('object_seen')) and node.moving and node.validation_samples > 0 and node.invalid_count == 0 and node.boundary_violations == 0 and
                            all(v is None or v > 0 for v in clearances.values()) and
                            (args.scenario == 'baseline' or node.b_seen) and
                            ((not arrived and abs(node.latest_speed)<0.05) if args.scenario == 'blocked' else arrived and node.complete_seen) and
                            (args.scenario not in ['passing','returning'] or node.extension > 0))
        if args.scenario in ['passing', 'returning', 'moving_then_static']:
            result['passed'] &= (not node.early_b_extension and node.first_b_extension_time is not None)
        if args.scenario == 'moving_then_static':
            result['passed'] &= (node.waited_for_b and node.b_stop_time is not None and
                                 node.b_perception_samples > 0 and
                                 all(node.clearance_samples.values()) and
                                 all(value is not None and value > 0 for value in clearances.values()) and
                                 node.extension > 0)
        (args.output_dir/'verdict.json').write_text(json.dumps(result,indent=2))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        figure, axes = plt.subplots(2, 1, figsize=(12, 7))
        if node.actual:
            axes[0].plot([p[0] for p in node.actual], [p[1] for p in node.actual], label='actual vehicle pose')
            axes[1].plot([t-node.actual_time_history[0] for t in node.actual_time_history],
                         [p[2] for p in node.actual])
        for name, obj in [('A',node.obstacle_msg),('B',node.b)]:
            if obj is not None:
                p = obj.initial_state.pose_covariance.pose.position
                axes[0].add_patch(Rectangle((p.x-obj.shape.dimensions.x/2,p.y-obj.shape.dimensions.y/2),
                                            obj.shape.dimensions.x,obj.shape.dimensions.y,alpha=0.4))
                axes[0].text(p.x,p.y,name)
        for y in [-2.9,1.65,6.15]:
            axes[0].axhline(y,color='gray',linestyle='--')
        axes[0].scatter([cfg['ego']['x'],cfg['goal']['x']], [cfg['ego']['y'],cfg['goal']['y']], marker='x', label='start / goal')
        axes[0].set(xlim=(min(cfg['ego']['x'],cfg['goal']['x'])-5,max(cfg['ego']['x'],cfg['goal']['x'])+5),ylim=(-4,8),xlabel='map x [m]',ylabel='map y [m]')
        axes[0].legend()
        axes[1].set(xlabel='elapsed time [s]',ylabel='actual speed [m/s]')
        figure.suptitle(args.scenario)
        figure.tight_layout()
        figure.savefig(args.output_dir/'actual_trajectory.png',dpi=150)
        plt.close(figure)
        print(json.dumps({k:v for k,v in result.items() if k != 'legacy_metrics'},indent=2))
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
