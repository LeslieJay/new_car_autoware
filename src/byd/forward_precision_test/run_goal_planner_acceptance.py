#!/usr/bin/env python3
"""Closed-loop goal approach acceptance using the real planning/control simulator.

Uses the repository's existing simulator setup driver; never injects trajectory or control.
Each invocation owns an isolated ROS domain and writes a result bag plus a verdict.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
DRIVERS = ROOT / 'src/byd/obstacle_avoidance_limit_test/scripts'
sys.path.insert(0, str(DRIVERS))
from run_campus_agv_acceptance import cleanup, track


def run_case(args):
    import rclpy
    from autoware_internal_debug_msgs.msg import StringStamped
    from autoware_planning_validator.msg import PlanningValidatorStatus
    from autoware_vehicle_msgs.msg import Engage, GearCommand
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from simple_avoidance_distance_test import DistanceNode, load_yaml

    cfg = load_yaml(ROOT / 'src/byd/obstacle_avoidance_limit_test/config/simple_avoidance_distance.yaml')
    cfg['ego'] = ({'x': -26.054, 'y': -13.319, 'z': -0.9, 'yaw': math.pi/2}
                  if args.case == 'turn' else {'x': -100., 'y': 3.88, 'z': 0., 'yaw': math.pi})
    cfg['goal'] = ({'x': -57.2748146057, 'y': 3.8158996105, 'z': -0.67458,
                    'yaw': math.radians(-177.7813)} if args.case == 'turn' else
                   {'x': -120., 'y': 3.912, 'z': 0., 'yaw': math.pi})
    cfg['test'].update(sample_sec=args.duration, obstacle_hold_sec=0)
    rclpy.init()

    class Scenario(DistanceNode):
        def __init__(self):
            self.armed = False
            self.first_moving = None
            self.goal_active = False
            self.first_goal_active_distance = None
            self.validation = []
            self.commands = []
            self.poses = []
            self.tracking = []
            super().__init__(cfg, 'avoidance_by_lane_change', {}, launch_log=args.output/'launch.log',
                             speed_limit_mps=2.0, trailer_types=[])
            self.engage = self.create_publisher(Engage, '/vehicle/engage', 10)
            self.direct_gear = self.create_publisher(GearCommand, '/control/command/gear_cmd',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            # An obstacle-free world needs a valid free-space grid even when the dummy LiDAR
            # produces an empty cloud; the laserscan mapper may otherwise publish nothing.
            self.grid_pub = self.create_publisher(OccupancyGrid, '/perception/occupancy_grid_map/map',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            self.grid = OccupancyGrid();self.grid.header.frame_id = 'map'
            self.grid.info.resolution = .2
            self.grid.info.origin.position.x = min(cfg['ego']['x'],cfg['goal']['x'])-10.
            self.grid.info.origin.position.y = min(cfg['ego']['y'],cfg['goal']['y'])-10.
            self.grid.info.origin.orientation.w = 1.
            self.grid.info.width = math.ceil((abs(cfg['ego']['x']-cfg['goal']['x'])+20.)/.2)
            self.grid.info.height = math.ceil((abs(cfg['ego']['y']-cfg['goal']['y'])+20.)/.2)
            self.grid.data = [0]*(self.grid.info.width*self.grid.info.height)
            self.create_timer(.2, self.publish_free_world)
            self.create_timer(.1, self.keep_enabled)
            self.create_subscription(StringStamped,
                '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state',
                self.internal_state, 10)
            self.create_subscription(PlanningValidatorStatus,
                '/planning/planning_validator/validation_status', self.validate, 100)

        def _call_service(self, client, request, wait_sec=10., timeout_sec=5.):
            # Map loading and route setup can outlast the generic driver's 5 s response
            # deadline. Do not treat merely receiving a response as successful setup.
            if not client.wait_for_service(timeout_sec=max(wait_sec,20.)):
                self.get_logger().error('Service unavailable: '+client.srv_name)
                return False
            future=client.call_async(request)
            rclpy.spin_until_future_complete(self,future,timeout_sec=max(timeout_sec,20.))
            if not future.done() or future.result() is None:
                self.get_logger().error('Service response timeout: '+client.srv_name)
                return False
            response=future.result()
            status=getattr(response,'status',None)
            if status is not None:
                ok=(status.success if hasattr(status,'success') else
                    status.code==status.SUCCESS if hasattr(status,'SUCCESS') else True)
                if not ok:
                    self.get_logger().warning(client.srv_name+': '+status.message)
                    return False
            return True

        def publish_free_world(self):
            self.grid.header.stamp = self.get_clock().now().to_msg()
            self.grid_pub.publish(self.grid)

        def keep_enabled(self):
            if not self.armed: return
            self.publish_speed_limit()
            self.publish_trailer_configuration()
            self.publish_drive_gear()
            msg = Engage();msg.stamp = self.get_clock().now().to_msg();msg.engage = True
            self.engage.publish(msg)
            gear = GearCommand();gear.stamp = msg.stamp;gear.command = GearCommand.DRIVE
            self.direct_gear.publish(gear)

        def internal_state(self, msg):
            if 'goal_planner(RUNNING)' in msg.data:
                if not self.goal_active:
                    g = cfg['goal'];self.first_goal_active_distance = math.hypot(
                        self.latest_pose[0]-g['x'], self.latest_pose[1]-g['y'])
                self.goal_active = True

        def validate(self, msg):
            if self.first_moving is not None:
                self.validation.append([name for name in msg.get_fields_and_field_types()
                    if name.startswith('is_valid') and not getattr(msg,name)])

        def _gate_control(self, msg):
            super()._gate_control(msg)
            if self.first_moving is not None: self.commands.append(time.monotonic())

        def _odom(self, msg):
            super()._odom(msg)
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if self.first_moving is None and abs(msg.twist.twist.linear.x)>.05:
                self.first_moving=time.monotonic()
            if self.first_moving is not None:
                self.poses.append([p.x,p.y,yaw,msg.twist.twist.linear.x])
                if len(self.latest_trajectory)>1:
                    from simple_avoidance_distance_test import project_to_reference
                    self.tracking.append(abs(project_to_reference(p.x,p.y,self.latest_trajectory)[1]))

    node=Scenario()
    try:
        if args.request_length is not None:
            from rcl_interfaces.srv import SetParameters
            from rclpy.parameter import Parameter
            client=node.create_client(SetParameters,
                '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/set_parameters')
            if not client.wait_for_service(timeout_sec=20.):
                raise RuntimeError('fixed-goal parameter service unavailable')
            request=SetParameters.Request(parameters=[Parameter(
                'goal_planner.fixed_goal.request_length',value=args.request_length).to_parameter_msg()])
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=20.)
            if not future.done() or not all(r.successful for r in future.result().results):
                raise RuntimeError('fixed-goal request-length override rejected')
        node.publish_setup()
        node.armed=True
        node.activate_vehicle(retries=3)
        deadline=time.monotonic()+args.duration;settled=None;arrived=False
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            g=cfg['goal'];distance=math.hypot(node.latest_pose[0]-g['x'],node.latest_pose[1]-g['y'])
            if node.first_moving is not None and distance<.1 and abs(node.latest_speed)<.02:
                if settled is None: settled=time.monotonic()
                if time.monotonic()-settled>2.0: arrived=True;break
            else: settled=None
        pose=node.poses[-1] if node.poses else [0,0,0,0]
        yaw_error=abs(math.atan2(math.sin(pose[2]-cfg['goal']['yaw']),math.cos(pose[2]-cfg['goal']['yaw'])))
        gaps=[b-a for a,b in zip(node.commands,node.commands[1:])]
        failures=[]
        if not arrived: failures.append('goal_not_reached_and_stopped')
        if not node.goal_active: failures.append('fixed_goal_module_not_exercised')
        if len(node.commands)<100: failures.append('insufficient_control_output')
        if max(gaps,default=999)>.5: failures.append('control_output_gap_over_500ms')
        if max(node.tracking,default=999)>.3: failures.append('tracking_error_over_30cm')
        if max((abs(p[3]) for p in node.poses),default=999)>2.05: failures.append('speed_limit_exceeded')
        if yaw_error>math.radians(5): failures.append('final_heading_error_over_5deg')
        non_latency=[field for fields in node.validation for field in fields if field!='is_valid_latency']
        if non_latency: failures.append('invalid_planning_geometry_or_dynamics')
        effective=args.request_length if args.request_length is not None else 3.
        if node.first_goal_active_distance is None or node.first_goal_active_distance>effective+.5:
            failures.append('fixed_goal_activated_before_request_range')
        result=dict(passed=not failures,failures=failures,case=args.case,
            request_length_m=effective,goal_reached=arrived,goal_module_seen=node.goal_active,
            first_activation_distance_m=node.first_goal_active_distance,
            final_distance_m=distance,final_yaw_error_deg=math.degrees(yaw_error),
            max_speed_mps=max((abs(p[3]) for p in node.poses),default=0),
            max_tracking_error_m=max(node.tracking,default=None),
            max_control_gap_s=max(gaps,default=None),control_samples=len(node.commands),
            validation_samples=len(node.validation),invalid_fields=sorted(set(non_latency)))
        (args.output/'verdict.json').write_text(json.dumps(result,indent=2))
        (args.output/'poses.json').write_text(json.dumps(node.poses))
        print(json.dumps(result,indent=2),flush=True)
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node();rclpy.shutdown()


def finalise_recorded(output):
    """Grade command continuity from the recorder, not a service-calling live observer."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from autoware_planning_validator.msg import PlanningValidatorStatus
    result=json.loads((output/'verdict.json').read_text())
    db=next((output/'bag').glob('*.db3'))
    con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
    topics={n:i for i,n in con.execute('select id,name from topics')}
    first=None
    for t,payload in con.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                                (topics['/localization/kinematic_state'],)):
        if abs(deserialize_message(payload,Odometry).twist.twist.linear.x)>.05:
            first=t/1e9;break
    if first is not None:
        stamps=[t/1e9 for (t,) in con.execute(
            'select timestamp from messages where topic_id=? and timestamp>=? order by timestamp',
            (topics['/control/command/control_cmd'],int(first*1e9)))]
        gaps=[b-a for a,b in zip(stamps,stamps[1:])]
        result.setdefault('live_subscriber_max_control_gap_s',result.get('max_control_gap_s'))
        result['max_control_gap_s']=max(gaps,default=None)
        result['recorded_control_samples']=len(stamps)
        result['control_continuity_source']='recorded_gate_output_after_motion_start'
        invalid=set();count=0
        for t,payload in con.execute('select timestamp,data from messages where topic_id=? and timestamp>=?',
                (topics['/planning/planning_validator/validation_status'],int(first*1e9))):
            m=deserialize_message(payload,PlanningValidatorStatus);count+=1
            invalid.update(k for k in m.get_fields_and_field_types()
                if k.startswith('is_valid') and k!='is_valid_latency' and not getattr(m,k))
        result['recorded_validation_samples']=count
        if count<10 and 'insufficient_recorded_validation_samples' not in result['failures']:
            result['failures'].append('insufficient_recorded_validation_samples')
        result['invalid_fields']=sorted(invalid)
        if len(stamps)>=100 and gaps and max(gaps)<=.5:
            result['failures']=[f for f in result['failures'] if f!='control_output_gap_over_500ms']
        elif 'control_output_gap_over_500ms' not in result['failures']:
            result['failures'].append('control_output_gap_over_500ms')
        if invalid and 'invalid_planning_geometry_or_dynamics' not in result['failures']:
            result['failures'].append('invalid_planning_geometry_or_dynamics')
    result['passed']=not result['failures']
    (output/'verdict.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['turn','straight'],default='turn')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain',type=int,default=63)
    parser.add_argument('--duration',type=float,default=100.)
    parser.add_argument('--request-length',type=float)
    parser.add_argument('--scenario-only',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--analyse-only',action='store_true')
    args=parser.parse_args();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    if args.analyse_only: return finalise_recorded(args.output)
    if args.scenario_only: return run_case(args)
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',
             RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_LOG_DIR=str(args.output/'roslog'))
    launch_cmd=['ros2','launch','autoware_launch','planning_simulator.launch.xml',
                'avoidance_by_lane_change_profile:=campus_agv','initial_engage_state:=false',
                'enable_all_modules_auto_mode:=true','perception/enable_detection_failure:=false',
                'rviz:=false','launch_static_obstacle_avoidance:=false',
                'launch_simple_avoidance:=false','launch_simple_lc_avoidance:=false']
    record_cmd=['ros2','bag','record','-o',str(args.output/'bag'),
                '/localization/kinematic_state','/planning/trajectory','/control/command/control_cmd',
                '/planning/planning_validator/validation_status',
                '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
                '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state']
    (args.output/'commands.json').write_text(json.dumps(dict(launch=launch_cmd,record=record_cmd,domain=args.domain),indent=2))
    children={};launch=recorder=scenario=None
    try:
        with (args.output/'launch.log').open('w') as log:
            launch=subprocess.Popen(launch_cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            until=time.monotonic()+360;ready=False
            while launch.poll() is None and time.monotonic()<until:
                track(launch,children)
                if "Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" in (args.output/'launch.log').read_text(errors='replace'):
                    ready=True;break
                time.sleep(.5)
            if not ready: raise RuntimeError('planning_simulator did not load BPP')
            with (args.output/'record.log').open('w') as record_log:
                recorder=subprocess.Popen(record_cmd,cwd=ROOT,env=env,stdout=record_log,stderr=subprocess.STDOUT,start_new_session=True)
                command=[sys.executable,str(Path(__file__).resolve()),*sys.argv[1:],'--scenario-only']
                with (args.output/'scenario.log').open('w') as scenario_log:
                    scenario=subprocess.Popen(command,cwd=ROOT,env=env,stdout=scenario_log,stderr=subprocess.STDOUT,start_new_session=True)
                    until=time.monotonic()+args.duration+240
                    while scenario.poll() is None and time.monotonic()<until:
                        track(launch,children);time.sleep(.5)
                    if scenario.poll() is None: raise RuntimeError('scenario timeout')
                    cleanup(recorder,{});recorder=None
                    return finalise_recorded(args.output)
    finally:
        for proc in [scenario,recorder]:
            if proc and proc.poll() is None: cleanup(proc,{})
        if launch: cleanup(launch,children)

if __name__=='__main__':
    raise SystemExit(main())
