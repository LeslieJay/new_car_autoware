#!/usr/bin/env python3
"""A single fresh-instance closed-loop case. No direct vehicle control bypass."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import yaml

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from autoware_internal_debug_msgs.msg import StringStamped
from autoware_adapi_v1_msgs.msg import RouteState, RouteOption, OperationModeState
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from tier4_rtc_msgs.msg import CooperateStatusArray
from autoware_planning_validator.msg import PlanningValidatorStatus
from autoware_perception_msgs.msg import ObjectClassification
from tier4_simulation_msgs.msg import DummyObject
from rcl_interfaces.srv import ListParameters, GetParameters
from rclpy.parameter import parameter_value_to_python

from simple_avoidance_distance_test import DistanceNode, PlanningPath, load_yaml
from dummy_object_utils import make_dummy_object, yaw_to_quaternion
from dual_avoidance_common import footprint, inside, manager_state, rectangle_distance, verdict, point_segment, simplify
from dual_avoidance_suite import BASE, BPP, write_json, stop_process


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class DualScenario(DistanceNode):
    def __init__(self,case,map_data,output,group):
        self.case,self.map_data,self.output,self.group = case,map_data,output,group
        self.timeline = (output/'timeline.jsonl').open('w',buffering=1)
        self.last_events = {}
        self.manager_samples = 0
        self.executed = set()
        self.registered = set()
        self.approved = []
        self.candidates = []
        self.geometry_samples = 0
        self.collision = False
        self.boundary_violations = 0
        self.min_clearance = None
        self.min_boundary_clearance = None
        self.opposite_entered = False
        self.returned = False
        self.objects_verified = set()
        self.injections = []
        self.event_fired = False
        self.released = False
        self.phase = 'setup'
        self.started = None
        self.last_geometry_stamp = -math.inf
        self.validator_samples = 0
        self.validator_invalid = 0
        self.validator_non_latency_invalid = 0
        self.refusal_evidence = False
        self.stop_since = None
        self.max_stop_duration = 0.0
        self.environment_errors = []
        self.state_transitions = 0
        self.processing_times = {}
        self.borrowing_complete = False
        self.raw_last = None
        self.new_running_at = None
        self.all_passed_once = False
        self.autonomous_seen=False
        self.injection_speed=None
        self.ego_geometry = {'rear':0.676,'front':1.554,'half_width':0.6525}
        self.road_polygons=[]
        boundary_uses={}
        for lane in map_data['lanes'].values():
            poly=simplify(lane['left'])+list(reversed(simplify(lane['right'])))
            self.road_polygons.append((poly,(min(p[0] for p in poly),max(p[0] for p in poly),
                                            min(p[1] for p in poly),max(p[1] for p in poly))))
            for side in ('left','right'):
                boundary_uses.setdefault(lane['members'][side],[]).append(simplify(lane[side]))
        self.outer_segments=[(a,b) for uses in boundary_uses.values() if len(uses)==1
                             for a,b in zip(uses[0],uses[0][1:])]
        cfg = load_yaml(BASE/'config/simple_avoidance_distance.yaml')
        cfg.update(ego=case['ego'],goal=case['goal'])
        super().__init__(cfg,'avoidance_by_lane_change',{},'start_lock',output/'launch.log',
                         speed_limit_mps=case['speed'],trailer_types=[],case_name=case['name'])
        self.reset_pose_ready = False
        self.armed = False
        self.create_subscription(StringStamped,BPP+'/debug/internal_state',self.manager,10)
        self.create_subscription(AvoidanceDebugMsgArray,BPP+'/debug/avoidance_debug_message_array',self.avoidance_debug,10)
        self.create_subscription(PlanningValidatorStatus,'/planning/planning_validator/validation_status',self.validator,10)
        self.create_subscription(OperationModeState,'/api/operation_mode/state',self.operation_mode,
                                 QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for module in ('static_obstacle_avoidance','avoidance_by_lane_change','lane_change_left','lane_change_right',
                       'obstacle_stop','dynamic_obstacle_stop','goal_planner'):
            self.create_subscription(PlanningFactorArray,'/planning/planning_factors/'+module,
                                     lambda m,name=module:self.factors(name,m),10)
            self.create_subscription(PlanningPath,'/planning/path_candidate/'+module,
                                     lambda m,name=module:self.candidate(name,m),10)
        for name in ('avoidance_left','avoidance_right','static_obstacle_avoidance_left','static_obstacle_avoidance_right',
                     'avoidance_by_lane_change_left','avoidance_by_lane_change_right'):
            self.create_subscription(CooperateStatusArray,'/planning/cooperate_status/'+name,
                                     lambda m,name=name:self.rtc(name,m),10)

    def emit(self,event,value,changed=True):
        if changed and self.last_events.get(event)==value:
            return
        self.last_events[event] = value
        self.timeline.write(json.dumps(dict(time=time.time(),ros_time=self.get_clock().now().nanoseconds/1e9,
                                            phase=self.phase,event=event,value=value),ensure_ascii=False)+'\n')

    def manager(self,msg):
        data = manager_state(msg.data)
        self.manager_samples += 1
        self.registered.update(n for n,s in data['registered'])
        self.approved = [(n,s) for n,s in data['approved']]
        self.candidates = [(n,s) for n,s in data['candidate']]
        running = [n for n,s in self.approved if s=='RUNNING']
        self.executed.update(running)
        if running and abs(self.latest_speed)>0.05 and self.new_running_at is None and any(n in ('static_obstacle_avoidance','avoidance_by_lane_change') for n in running):
            self.new_running_at = time.monotonic()
        if self.raw_last != data:
            self.state_transitions += 1
        self.raw_last = data
        self.emit('manager',data)
        if 'update module info:' in msg.data:
            info = msg.data.split('update module info:')[1].split('processing time')[0].strip()
            self.emit('manager_actions',info)
        for name,value in re.findall(r'\[\s*([a-z_]+)\s*:\s*([\d.]+)ms\]',msg.data):
            self.processing_times[name] = max(self.processing_times.get(name,0),float(value))

    def avoidance_debug(self,msg):
        info = [message_to_ordereddict(m) for m in msg.avoidance_info]
        self.emit('static_object_filtering',info)
        self.refusal_evidence |= any(m.failed_reason for m in msg.avoidance_info)

    def factors(self,name,msg):
        data = message_to_ordereddict(msg)
        # Header stamps vary each publication; deduplicate the actual factors.
        value = data.get('factors',data)
        self.emit('factors/'+name,value)
        self.refusal_evidence |= bool(getattr(msg,'factors',[])) and name in ('obstacle_stop','dynamic_obstacle_stop')

    def candidate(self,name,msg):
        self.emit('candidate/'+name,dict(nonempty=bool(msg.points),points=len(msg.points)))

    def rtc(self,name,msg):
        value = [dict(uuid=bytes(s.uuid.uuid).hex(),requested=s.requested,safe=s.safe,
                      state=s.state.type,command=s.command_status.type,auto=s.auto_mode) for s in msg.statuses]
        self.emit('rtc/'+name,value)
        self.refusal_evidence |= any(s.requested and not s.safe for s in msg.statuses)

    def validator(self,msg):
        if self.phase != 'running':
            return
        self.validator_samples += 1
        invalid = [name for name in msg.get_fields_and_field_types() if name.startswith('is_valid') and getattr(msg,name) is False]
        # Latency is retained as a failure too, not silently excluded.
        self.validator_invalid += bool(invalid)
        self.validator_non_latency_invalid += any(field!='is_valid_latency' for field in invalid)
        self.emit('validator_invalid_fields',invalid)

    def operation_mode(self,msg):
        self.autonomous_seen |= msg.mode==OperationModeState.AUTONOMOUS and msg.is_autoware_control_enabled
        self.emit('operation_mode',dict(mode=msg.mode,control_enabled=msg.is_autoware_control_enabled,
                                       transition=msg.is_in_transition))

    def _rosout(self,msg):
        if '[OPPOSITE_BORROWING]' in msg.msg:
            fields = dict(re.findall(r'([a-z_]+)=([^\s]+)',msg.msg))
            self.borrowing_complete |= fields.get('phase')=='complete'
            self.refusal_evidence |= fields.get('safe')=='0'
            # Keep full diagnostics in bag; timeline retains state/reason transitions.
            self.emit('opposite_borrowing',{k:fields[k] for k in ('phase','safe','reason','requested') if k in fields})

    def _objects(self,msg):
        self.latest_objects = msg
        if self.phase!='running':
            return
        for index,item in enumerate(self.injections):
            if not item['active']:
                continue
            x,y,_ = self.object_pose(item)
            for obj in msg.objects:
                p = obj.kinematics.initial_pose_with_covariance.pose.position
                if math.hypot(p.x-x,p.y-y)<1.5:
                    self.objects_verified.add(index)
                    self.emit('object_seen/'+str(index),bytes(obj.object_id.uuid).hex())

    def object_pose(self,item):
        pose = item['spec']['pose']
        dt = max(0,self.get_clock().now().nanoseconds/1e9-item['stamp'])
        speed = item['speed']
        return pose['x']+speed*dt*math.cos(pose['yaw']),pose['y']+speed*dt*math.sin(pose['yaw']),pose['yaw']

    def _odom(self,msg):
        super()._odom(msg)
        if self.phase!='running':
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp-self.last_geometry_stamp<0.045:
            return
        self.last_geometry_stamp = stamp
        p = msg.pose.pose.position
        angle = yaw_of(msg.pose.pose.orientation)
        body = footprint(p.x,p.y,angle,**self.ego_geometry)
        self.geometry_samples += 1
        # Sample all body edges as well as vertices to detect bodies spanning non-road gaps.
        edge_points = []
        for a,b in zip(body,body[1:]+body[:1]):
            steps = max(1,math.ceil(math.dist(a,b)/0.1))
            edge_points.extend((a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps) for i in range(steps+1))
        outside = any(not any(box[0]-1e-6<=q[0]<=box[1]+1e-6 and box[2]-1e-6<=q[1]<=box[3]+1e-6 and inside(q,poly)
                              for poly,box in self.road_polygons) for q in edge_points)
        boundary_clearance=min(point_segment(q,a,b) for q in body for a,b in self.outer_segments)
        self.min_boundary_clearance=min(self.min_boundary_clearance,boundary_clearance) if self.min_boundary_clearance is not None else boundary_clearance
        self.boundary_violations += int(outside)
        lane = self.map_data['lanes'][self.case['lane']]
        if 'goal_lane' not in self.case['route']:
            # Only classify opposite occupancy on the validated straight lane pair.
            pair = [p for p in self.map_data['shared_boundary_pairs'] if self.case['lane'] in p['lanes'] and p['direction']=='opposite']
            opposite_polygons = [self.map_data['lanes'][n]['polygon'] for pair_item in pair
                                 for n in pair_item['lanes'] if n!=self.case['lane']]
            self.opposite_entered |= any(inside(q,poly) for q in body for poly in opposite_polygons)
            if self.opposite_entered and all(inside(q,lane['polygon']) for q in body):
                self.returned = True
            elif any(inside(q,poly) for q in body for poly in opposite_polygons):
                self.returned = False
        passed = True
        for item in self.injections:
            if not item['active']:
                continue
            x,y,yaw = self.object_pose(item)
            spec = item['spec']
            obstacle = footprint(x,y,yaw,spec['length']/2,spec['length']/2,spec['width']/2)
            distance = rectangle_distance(body,obstacle)
            self.min_clearance = min(self.min_clearance,distance) if self.min_clearance is not None else distance
            self.collision |= distance<=1e-6
            if spec['initially_ahead']:
                heading = self.case['ego']['yaw']
                # Full rear edge must have cleared the obstacle's farthest forward corner.
                rear_s = min(q[0]*math.cos(heading)+q[1]*math.sin(heading) for q in body)
                far_s = max(q[0]*math.cos(heading)+q[1]*math.sin(heading) for q in obstacle)
                passed &= rear_s>far_s
        self.all_passed_once = passed
        if abs(self.latest_speed)<0.05:
            self.stop_since = self.stop_since or time.monotonic()
            self.max_stop_duration = max(self.max_stop_duration,time.monotonic()-self.stop_since)
        else:
            self.stop_since = None
        self.emit('safety',dict(collision=self.collision,outside=outside,passed_all=passed))

    def _call_service(self,client,request,wait_sec=10,timeout_sec=5):
        if not client.wait_for_service(timeout_sec=wait_sec):
            self.emit('service/'+client.srv_name,{'available':False})
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self,future,timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        status = getattr(response,'status',None)
        success = response is not None and getattr(status,'success',getattr(response,'success',True))
        self.emit('service/'+client.srv_name,dict(success=success,code=getattr(status,'code',None),message=getattr(status,'message','')))
        return success

    def spin_for(self,seconds):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            self.publish_speed_limit()
            self.publish_trailer_configuration()
            rclpy.spin_once(self,timeout_sec=0.05)

    def snapshot_parameters(self,name):
        listing=self.create_client(ListParameters,name+'/list_parameters')
        getting=self.create_client(GetParameters,name+'/get_parameters')
        try:
            if not listing.wait_for_service(timeout_sec=10) or not getting.wait_for_service(timeout_sec=10):
                raise RuntimeError('parameter service unavailable')
            future=listing.call_async(ListParameters.Request())
            rclpy.spin_until_future_complete(self,future,timeout_sec=10)
            if not future.done() or future.result() is None:
                raise RuntimeError('list_parameters timeout')
            names=future.result().result.names
            request=GetParameters.Request(names=names)
            future=getting.call_async(request)
            rclpy.spin_until_future_complete(self,future,timeout_sec=10)
            if not future.done() or future.result() is None:
                raise RuntimeError('get_parameters timeout')
            params={key:parameter_value_to_python(value) for key,value in zip(names,future.result().values)}
            path=self.output/(name.rsplit('/',1)[1]+'.params.yaml')
            path.write_text(yaml.safe_dump({name:{'ros__parameters':params}},sort_keys=True))
            return params
        finally:
            self.destroy_client(listing); self.destroy_client(getting)

    def verify_parameters(self,params):
        from ament_index_python.packages import get_package_share_directory
        from dual_avoidance_common import file_hash
        root=Path(get_package_share_directory('autoware_launch'))/'config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'
        sources=['scene_module_manager.param.yaml',
                 'autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml',
                 'avoidance_by_lane_change/avoidance_by_lane_change.param.yaml']
        def flat(value,prefix=''):
            result={}
            for key,item in value.items():
                name=prefix+key
                if isinstance(item,dict):
                    result.update(flat(item,name+'.'))
                else:
                    result[name]=item
            return result
        checks=[]
        for source in sources:
            path=root/source
            expected=flat(load_yaml(path)['/**']['ros__parameters'])
            mismatches={key:dict(source=value,runtime=params[key]) for key,value in expected.items()
                        if key in params and params[key]!=value}
            checks.append(dict(path=str(path),sha256=file_hash(path),compared=sum(k in params for k in expected),
                               not_declared=[k for k in expected if k not in params],mismatches=mismatches))
        write_json(self.output/'parameter_comparison.json',checks)
        if any(item['mismatches'] for item in checks):
            self.environment_errors.append('Runtime/source parameter mismatch; see parameter_comparison.json')

    def resolve_thresholds(self):
        if 'object_speed_threshold_offset' not in self.case:
            return
        namespace='avoidance' if self.group=='A' else 'avoidance_by_lane_change'
        key=namespace+'.target_object.car.'
        params=self.runtime_params
        threshold=params[key+'th_moving_speed']
        self.case['objects'][0]['speed']=max(0,threshold+self.case['object_speed_threshold_offset'])
        if 'motion_duration_threshold_offset' in self.case:
            self.case['event_after']=max(0.1,params[key+'th_moving_time']+self.case['motion_duration_threshold_offset'])
        self.emit('resolved_thresholds',dict(namespace=namespace,speed_threshold=threshold,
            moving_time_threshold=params[key+'th_moving_time'],speed=self.case['objects'][0]['speed'],
            event_after=self.case.get('event_after')))
        write_json(self.output/'effective_case.json',self.case)

    def setup(self,timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline and not self.initial_pub.get_subscription_count():
            rclpy.spin_once(self,timeout_sec=0.1)
        if not self.initial_pub.get_subscription_count():
            self.environment_errors.append('Initial pose subscriber unavailable'); return False
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        p = self.case['ego']
        pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z = p['x'],p['y'],p['z']
        pose.pose.pose.orientation = yaw_to_quaternion(p['yaw'])
        pose.pose.covariance[0]=pose.pose.covariance[7]=0.25
        pose.pose.covariance[35]=0.0685
        for _ in range(3):
            pose.header.stamp = self.get_clock().now().to_msg()
            self.initial_pub.publish(pose)
            self.spin_for(0.5)
        while time.monotonic()<deadline and not self.actual:
            # A topic monitor can match before the simulator subscribes. A volatile
            # late subscriber does not receive the publisher's cached pose. Retry
            # only before odometry/route hand-off, never while driving.
            pose.header.stamp = self.get_clock().now().to_msg()
            self.initial_pub.publish(pose)
            self.emit('initial_pose_retry',dict(subscribers=self.initial_pub.get_subscription_count()),False)
            self.spin_for(0.5)
        if not self.actual:
            self.environment_errors.append('No simulator odometry'); return False
        request = SetRoutePoints.Request()
        request.header.frame_id='map'
        request.option = RouteOption(allow_goal_modification=False)
        g = self.case['goal']
        request.goal = Pose()
        request.goal.position.x,request.goal.position.y,request.goal.position.z = g['x'],g['y'],g['z']
        request.goal.orientation = yaw_to_quaternion(g['yaw'])
        while time.monotonic()<deadline:
            request.header.stamp = self.get_clock().now().to_msg()
            if self.route_state!=RouteState.SET:
                self._call_service(self.route_service,request,wait_sec=2,timeout_sec=5)
            self.spin_for(2)
            if self.route_state==RouteState.SET and self.latest_behavior and self.latest_trajectory and self.manager_samples:
                return True
        self.environment_errors.append('Route/path/trajectory/manager not ready'); return False

    def spawn(self,spec):
        pose=spec['pose']
        obj=make_dummy_object('map',self.get_clock().now().to_msg(),pose['x'],pose['y'],pose['z'],pose['yaw'],
            length=spec['length'],width=spec['width'],height=spec['height'],velocity=spec['speed'],
            label=getattr(ObjectClassification,spec['label'].upper()))
        self.object_pub.publish(obj)
        item = dict(spec=spec,msg=obj,stamp=self.get_clock().now().nanoseconds/1e9,speed=spec['speed'],active=True)
        self.emit('object_injected',dict(index=len(self.injections),uuid=bytes(obj.id.uuid).hex(),spec=spec),False)
        self.injections.append(item)

    def change_object(self,index,remove=False,speed=None):
        item = self.injections[index]
        x,y,yaw = self.object_pose(item)
        msg=item['msg']
        msg.header.stamp = self.get_clock().now().to_msg()
        if remove:
            msg.action=DummyObject.DELETE
            item['active']=False
        else:
            item['spec']['pose'].update(x=x,y=y,yaw=yaw)
            msg.initial_state.pose_covariance.pose.position.x=x
            msg.initial_state.pose_covariance.pose.position.y=y
            msg.initial_state.twist_covariance.twist.linear.x=float(speed)
            msg.action=DummyObject.MODIFY
            item['speed']=speed
            item['stamp']=self.get_clock().now().nanoseconds/1e9
        self.object_pub.publish(msg)
        self.emit('object_changed',dict(index=index,removed=remove,speed=speed),False)

    def scenario_event(self):
        if self.event_fired:
            return
        event = self.case.get('event')
        elapsed = time.monotonic()-self.started
        fire = ((event in ('release_secondary','stop_target','move_target') and elapsed>=self.case.get('event_after',15)) or
                (event=='remove_before_execution' and elapsed>=3 and abs(self.latest_speed)<0.05) or
                (event in ('remove_running','add_running') and self.new_running_at is not None and
                 time.monotonic()-self.new_running_at>=2))
        if not fire:
            return
        if event=='release_secondary':
            self.change_object(1,remove=True); self.released=True
        elif event in ('remove_before_execution','remove_running'):
            self.change_object(0,remove=True); self.released=True
        elif event=='add_running':
            self.spawn(self.case['objects'][1])
        elif event=='stop_target':
            self.change_object(0,speed=0.0)
        elif event=='move_target':
            self.change_object(0,speed=0.5)
        self.event_fired=True
        self.emit('scenario_event',event)

    def save_plot(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,1,figsize=(12,7))
        for lane in self.map_data['lanes'].values():
            for side in ('left','right'):
                axes[0].plot([p[0] for p in lane[side]],[p[1] for p in lane[side]],color='gray',linewidth=0.5)
        if self.actual:
            axes[0].plot([p[0] for p in self.actual],[p[1] for p in self.actual],label='actual')
            axes[1].plot([t-self.actual_time_history[0] for t in self.actual_time_history],[p[2] for p in self.actual])
        for item in self.injections:
            p=item['spec']['pose']
            poly=footprint(p['x'],p['y'],p['yaw'],item['spec']['length']/2,item['spec']['length']/2,item['spec']['width']/2)
            axes[0].fill([p[0] for p in poly],[p[1] for p in poly],alpha=0.3)
        points=[self.case[k] for k in ('ego','goal')]
        axes[0].set(xlim=(min(p['x'] for p in points)-15,max(p['x'] for p in points)+15),
                    ylim=(min(p['y'] for p in points)-10,max(p['y'] for p in points)+10),xlabel='map x [m]',ylabel='map y [m]')
        axes[0].legend(); axes[0].set_aspect('equal',adjustable='datalim')
        axes[1].set(xlabel='elapsed [s]',ylabel='measured speed [m/s]')
        fig.suptitle(f"{self.case['name']} / {self.group}")
        fig.tight_layout(); fig.savefig(self.output/'trajectory.png',dpi=140); plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--map-inventory',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--group',required=True)
    parser.add_argument('--startup-timeout',type=float,default=180)
    args=parser.parse_args()
    case=json.loads(args.case.read_text())
    rclpy.init()
    node=DualScenario(case,json.loads(args.map_inventory.read_text()),args.output,args.group)
    recorder=None
    record_log=(args.output/'bag_record.log').open('w')
    data={}
    try:
        regex = ('/planning/(path_candidate/.*|path_reference/.*|planning_factors/.*|cooperate_status/.*|'
                 'scenario_planning/lane_driving/behavior_planning/.*|trajectory|turn_indicators_cmd|planning_validator/validation_status)|'
                 '/perception/object_recognition/objects|/simulation/dummy_perception_publisher/object_info|'
                 '/localization/(kinematic_state|acceleration)|/control/(command/.*|trajectory_follower/control_cmd)|'
                 '/vehicle/status/.*|/api/(routing/state|operation_mode/state)|/rosout|/tf|/tf_static')
        recorder=subprocess.Popen(['ros2','bag','record','--storage','sqlite3','-o',str(args.output/'bag'),'-e',regex],
            stdout=record_log,stderr=subprocess.STDOUT,start_new_session=True)
        if node.setup(args.startup_timeout):
            # Dump the actual node's declared parameters, not only source YAML.
            for name in (BPP,'/simulation/simple_planning_simulator'):
                try:
                    params=node.snapshot_parameters(name)
                    if name==BPP:
                        node.runtime_params=params
                        node.verify_parameters(params)
                except RuntimeError as error:
                    node.environment_errors.append(f'Runtime parameter snapshot failed: {name}: {error}')
            node.phase='running'
            node.case_start_epoch=node.get_clock().now().nanoseconds/1e9
            node.started=time.monotonic()
            node.resolve_thresholds()
            if case.get('lock_mode')=='steady_speed':
                node.set_autonomous(engage=True)
                steady_since=None
                deadline=time.monotonic()+45
                while time.monotonic()<deadline:
                    node.publish_drive_gear(); node.spin_for(0.1)
                    if abs(node.latest_speed-case['speed'])<=max(0.03,0.1*case['speed']):
                        steady_since=steady_since or time.monotonic()
                        if time.monotonic()-steady_since>=1:
                            break
                    else:
                        steady_since=None
                if steady_since is None or time.monotonic()-steady_since<1:
                    node.environment_errors.append('Requested steady speed was not attained before injection')
                dx,dy=(node.latest_pose[i]-case['ego'][k] for i,k in enumerate(('x','y')))
                for spec in case['objects']:
                    spec['pose']['x']+=dx; spec['pose']['y']+=dy
                node.emit('steady_speed_anchor',dict(x=node.latest_pose[0],y=node.latest_pose[1],speed=node.latest_speed))
            node.injection_speed=node.latest_speed
            for spec in case['objects']:
                if not spec['delay_until_running']:
                    node.spawn(spec)
            for _ in range(30):
                node.spin_for(0.1)
                node.scenario_event()
            node.scenario_event()
            node.set_autonomous(engage=True)
            deadline=time.monotonic()+case['timeout']
            while rclpy.ok() and time.monotonic()<deadline:
                node.scenario_event()
                node.publish_drive_gear()
                node.spin_for(0.1)
                if node.route_state==RouteState.ARRIVED:
                    node.spin_for(1); break
                if node.collision or node.boundary_violations:
                    break
                if node.stop_since is not None and time.monotonic()-node.stop_since>=30 and node.refusal_evidence and \
                   (not case.get('event') or node.event_fired) and all(item['speed']==0 for item in node.injections if item['active']):
                    break
            expected={'static_obstacle_avoidance':args.group!='B','avoidance_by_lane_change':args.group!='A'}
            for name,enabled in expected.items():
                if (name in node.registered)!=enabled:
                    node.environment_errors.append(f'Loaded module mismatch: {name}, expected enabled={enabled}')
        stop_process(recorder); recorder=None; record_log.flush()
        from campus_agv_scenario import recorded_validation
        recorded=recorded_validation(args.output/'bag',node.case_start_epoch)
        # Live counts cover only the test phase. Recorded counts are retained for auditing.
        bag_valid=(args.output/'bag/metadata.yaml').exists() and any((args.output/'bag').glob('*.db3'))
        if node.validator_samples==0:
            node.environment_errors.append('No planning validator samples during test')
        if recorded['invalid_count'] and not recorded['non_latency_invalid_count'] and not node.validator_non_latency_invalid:
            node.environment_errors.append('Planning timing check failed; environment retry required; samples retained')
        if not node.autonomous_seen:
            node.environment_errors.append('Autonomous control never confirmed by operation mode state')
        log=(args.output/'launch.log').read_text(errors='replace')
        process_failures=[dict(process=name,pid=int(pid),exit_code=int(code)) for name,pid,code in
            re.findall(r'\[ERROR\] \[([^\]]+)\]: process has died \[pid (\d+), exit code (-?\d+)',log)]
        for failure in process_failures:
            # This service handles manual route changes, which these fixed-route
            # cases never invoke. Keep its independent crash visible in results.
            if not failure['process'].startswith('autoware_manual_lane_change_handler_node-'):
                node.environment_errors.append('Required process exited: '+failure['process'])
        data=dict(scenario=case['name'],group=args.group,environment_errors=node.environment_errors,
            process_failures=process_failures,
            system_load_end=os.getloadavg(),cpu_count=os.cpu_count(),
            arrived=node.route_state==RouteState.ARRIVED,route_state=node.route_state,
            passed_all=node.all_passed_once,collision=node.collision,boundary_violations=node.boundary_violations,
            min_clearance=node.min_clearance,min_boundary_clearance=node.min_boundary_clearance,geometry_samples=node.geometry_samples,
            validator_invalid=max(node.validator_invalid,recorded['invalid_count']),validator_samples=recorded['samples'],
            validator_non_latency_invalid=max(node.validator_non_latency_invalid,recorded['non_latency_invalid_count']),
            recorded_validator=recorded,bag_valid=bool(bag_valid),
            manager_samples=node.manager_samples,executed_modules=sorted(node.executed),
            registered_modules=sorted(node.registered),state_transitions=node.state_transitions,
            processing_time_max_ms=node.processing_times,
            expected_objects=len(node.injections),objects_verified=len(node.objects_verified)==len(node.injections),
            opposite_entered=node.opposite_entered,returned=node.returned,borrowing_complete=node.borrowing_complete,
            released=node.released,refusal_evidence=node.refusal_evidence,
            stopped_safely=node.stop_since is not None and time.monotonic()-node.stop_since>=3 and
                           (node.min_clearance is None or node.min_clearance>0),
            control_ready=all(node.topic_seen.get(k,False) for k in ('gate_control','trajectory_follower_control')),
            autonomous_confirmed=node.autonomous_seen,
            scenario_event=case.get('event'),event_fired=node.event_fired,
            actual_speed_max=max((abs(p[2]) for p in node.actual),default=0),
            actual_injection_speed=node.injection_speed,
            final_pose=node.latest_pose,final_speed=node.latest_speed,
            request_observability='RTC requested flag when published; absent RTC is UNKNOWN, not false')
        data['result']=verdict(data)
        if case.get('event') and not node.event_fired and data['result'] in ('PASS','SAFE_STOP'):
            data['result']='INCONCLUSIVE'
            data['scenario_incomplete']='Required execution stage/event never occurred'
        write_json(args.output/'verdict.json',data)
        write_json(args.output/'actual.json',dict(samples=node.actual,times=node.actual_time_history))
        node.save_plot()
        print(json.dumps(data,ensure_ascii=False,indent=2),flush=True)
    finally:
        stop_process(recorder)
        record_log.close(); node.timeline.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if data.get('result') in ('PASS','SAFE_STOP') else 1


if __name__=='__main__':
    raise SystemExit(main())
