#!/usr/bin/env python3
"""Run real planning/control nodes against an input-only bag and check regenerated outputs."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/byd/obstacle_avoidance_limit_test/scripts'))
from run_campus_agv_acceptance import cleanup, track


def analyse(output, request_length):
    import sqlite3
    import math
    import bisect
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    db=next((output/'bag').glob('*.db3'))
    c=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
    topics={i:(n,get_message(ty)) for i,n,ty in c.execute('select id,name,type from topics')}
    samples={n:[] for n,_ in topics.values()}
    for tid,t,p in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
        n,cls=topics[tid];samples[n].append((t/1e9,deserialize_message(p,cls)))
    od=samples.get('/localization/kinematic_state',[]);ot=[t for t,m in od]
    traj=samples.get('/planning/trajectory',[])
    commands=samples.get('/control/trajectory_follower/control_cmd',[])
    gate_commands=samples.get('/control/command/control_cmd',[])
    validation=samples.get('/planning/planning_validator/validation_status',[])
    states=samples.get('/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state',[])
    failures=[]
    first=ot[0] if ot else 0
    invalid={};distances=[];activation=[]
    for t,m in validation:
        if t<first+5:continue
        for name in m.get_fields_and_field_types():
            if name.startswith('is_valid') and name!='is_valid_latency' and not getattr(m,name):
                invalid[name]=invalid.get(name,0)+1
    for t,m in traj:
        if t<first+5 or len(m.points)<2 or not od:continue
        _,ego=od[max(0,bisect.bisect_right(ot,t)-1)];p=ego.pose.pose.position
        best=float('inf')
        for a,b in zip(m.points,m.points[1:]):
            a=a.pose.position;b=b.pose.position;dx=b.x-a.x;dy=b.y-a.y
            f=max(0.,min(1.,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy+1e-30)))
            best=min(best,math.hypot(p.x-a.x-f*dx,p.y-a.y-f*dy))
        distances.append(best)
    for t,m in states:
        if 'goal_planner(RUNNING)' in m.data and od:
            p=od[max(0,bisect.bisect_right(ot,t)-1)][1].pose.pose.position
            activation.append(math.hypot(p.x+57.2748146057,p.y-3.8158996105))
    stamps=[t for t,m in commands if t>first+5]
    gaps=[b-a for a,b in zip(stamps,stamps[1:])]
    gate_stamps=[t for t,m in gate_commands if t>first+5]
    gate_gaps=[b-a for a,b in zip(gate_stamps,gate_stamps[1:])]
    text=(output/'launch.log').read_text(errors='replace')
    lost_control=len(re.findall(r'failed to solve pure_pursuit for control command calculation|Lateral control command too old',text))
    if len(traj)<500:failures.append('insufficient_regenerated_trajectories')
    if len(commands)<1000:failures.append('insufficient_regenerated_control')
    if len(gate_commands)<1000:failures.append('insufficient_regenerated_gate_control')
    if max(gate_gaps,default=999)>.5:failures.append('gate_control_gap_over_500ms')
    if invalid:failures.append('invalid_geometry_or_dynamics')
    # Open-loop replay retains the original manually displaced ego poses. The campus
    # execution-position tolerance is 0.75 m; closed-loop tracking is checked separately
    # at 0.30 m. Never grade open-loop poses as if they followed regenerated controls.
    if max(distances,default=999)>.75:failures.append('trajectory_distance_over_75cm')
    if max(gaps,default=999)>.5:failures.append('control_gap_over_500ms')
    if lost_control:failures.append('lateral_control_failure')
    if not activation:failures.append('fixed_goal_not_exercised')
    if activation and activation[0]>request_length+.5:failures.append('early_goal_activation')
    result=dict(passed=not failures,failures=failures,trajectory_samples=len(traj),control_samples=len(commands),
        gate_control_samples=len(gate_commands),max_gate_control_gap_s=max(gate_gaps,default=None),
        validator_samples=len(validation),invalid_fields=invalid,max_trajectory_distance_m=max(distances,default=None),
        max_control_gap_s=max(gaps,default=None),lateral_control_failures=lost_control,
        goal_activation_samples=len(activation),first_goal_activation_distance_m=activation[0] if activation else None,
        request_length_m=request_length,mode='open_loop_input_bag_replay',
        recorded_planning_and_control_outputs_replayed=False,
        open_loop_distance_limit_m=.75,control_gap_limit_s=.5)
    (output/'verdict.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain',type=int,default=64)
    parser.add_argument('--rate',type=float,default=1.0)
    parser.add_argument('--request-length',type=float,default=3.)
    parser.add_argument('--analyse-only',action='store_true')
    args=parser.parse_args();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    if args.analyse_only:return analyse(args.output,args.request_length)
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_LOG_DIR=str(args.output/'roslog'))
    launch_cmd=['ros2','launch','autoware_launch','logging_simulator.launch.xml',
        'avoidance_by_lane_change_profile:=campus_agv','enable_all_modules_auto_mode:=true',
        'vehicle:=false','system:=false','rviz:=false','perception:=false','didrive_perception:=false',
        'localization:=false','control:=true','planning:=true']
    outputs=['/localization/kinematic_state','/planning/trajectory',
        '/control/trajectory_follower/control_cmd','/control/command/control_cmd',
        '/planning/planning_validator/validation_status',
        '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
        '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state']
    record_cmd=['ros2','bag','record','-o',str(args.output/'bag'),*outputs]
    play_cmd=['ros2','bag','play',str(args.bag.resolve()),'--clock','100','--rate',str(args.rate),'--delay','2','--disable-keyboard-controls']
    config_cmd=['ros2','topic','pub','--rate','1','--qos-durability','transient_local',
        '/vehicle/status/trailer_configuration','byd_vehicle_msgs/msg/TrailerConfiguration','{trailer_types: []}']
    (args.output/'commands.json').write_text(json.dumps(dict(launch=launch_cmd,record=record_cmd,play=play_cmd,configuration=config_cmd,domain=args.domain),indent=2))
    children={};launch=player=recorder=config=None
    try:
        with (args.output/'launch.log').open('w') as log:
            launch=subprocess.Popen(launch_cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            until=time.monotonic()+360;ready=False
            while launch.poll() is None and time.monotonic()<until:
                track(launch,children)
                if "Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" in (args.output/'launch.log').read_text(errors='replace'):
                    ready=True;break
                time.sleep(.5)
            if not ready:raise RuntimeError('logging_simulator did not load BPP')
            if args.request_length!=3.:
                subprocess.run(['ros2','param','set','/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner','goal_planner.fixed_goal.request_length',str(args.request_length)],env=env,check=True,timeout=20)
            with (args.output/'record.log').open('w') as record_log,(args.output/'play.log').open('w') as play_log,(args.output/'configuration.log').open('w') as cfg_log:
                recorder=subprocess.Popen(record_cmd,cwd=ROOT,env=env,stdout=record_log,stderr=subprocess.STDOUT,start_new_session=True)
                config=subprocess.Popen(config_cmd,cwd=ROOT,env=env,stdout=cfg_log,stderr=subprocess.STDOUT,start_new_session=True)
                time.sleep(3)
                player=subprocess.Popen(play_cmd,cwd=ROOT,env=env,stdout=play_log,stderr=subprocess.STDOUT,start_new_session=True)
                until=time.monotonic()+330/args.rate+60
                while player.poll() is None and time.monotonic()<until:
                    track(launch,children);time.sleep(.5)
                if player.poll()!=0:raise RuntimeError('input bag playback did not finish cleanly')
                time.sleep(2);cleanup(recorder,{});recorder=None
                return analyse(args.output,args.request_length)
    finally:
        for proc in [player,config,recorder]:
            if proc and proc.poll() is None:cleanup(proc,{})
        if launch:cleanup(launch,children)

if __name__=='__main__':raise SystemExit(main())
