#!/usr/bin/env python3
"""Replay recorded inputs only; never replay old planning outputs into the new planner."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from run_campus_agv_acceptance import ROOT, cleanup, track


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, default=ROOT/'log/20261105/161119_bag')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=176)
    parser.add_argument('--rate', type=float, default=0.5)
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error('--rate must be positive')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=str(args.output_dir/'roslog'))
    launch_command = ['ros2','launch','autoware_launch','logging_simulator.launch.xml',
                      'avoidance_by_lane_change_profile:=campus_agv','enable_all_modules_auto_mode:=true',
                      'rviz:=false','control:=false','system:=false',
                      'launch_static_obstacle_avoidance:=false','launch_simple_avoidance:=false',
                      'launch_simple_lc_avoidance:=false']
    inputs = ['/localization/kinematic_state','/localization/acceleration',
              '/perception/object_recognition/objects','/perception/occupancy_grid_map/map',
              '/perception/obstacle_segmentation/pointcloud','/planning/mission_planning/route',
              '/vehicle/status/steering_status','/vehicle/status/gear_status','/vehicle/status/control_mode',
              '/vehicle/status/velocity_status','/system/operation_mode/state','/tf','/tf_static']
    play_command = ['ros2','bag','play',str(args.bag),'--clock','--rate',str(args.rate),'--topics',*inputs]
    outputs = ['/planning/path_candidate/avoidance_by_lane_change',
               '/planning/path_reference/avoidance_by_lane_change',
               '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
               '/planning/trajectory','/planning/planning_factors/avoidance_by_lane_change',
               '/planning/planning_factors/obstacle_stop','/planning/planning_validator/validation_status',
               '/localization/kinematic_state','/perception/object_recognition/objects',
               '/vehicle/status/trailer_configuration','/clock']
    record_command = ['ros2','bag','record','-o',str(args.output_dir/'bag'),*outputs]
    configuration_command = ['ros2','topic','pub','--rate','1','--qos-durability','transient_local',
        '/vehicle/status/trailer_configuration','byd_vehicle_msgs/msg/TrailerConfiguration','{trailer_types: []}']
    (args.output_dir/'commands.json').write_text(json.dumps({'launch':launch_command,'play':play_command,'record':record_command,'empty_trailer_configuration':configuration_command},indent=2))
    profile = ROOT/'src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/campus_agv.param.yaml'
    (args.output_dir/'effective_parameters.yaml').write_text(profile.read_text())
    common = profile.parents[1]/'behavior_path_planner.param.yaml'
    (args.output_dir/'effective_behavior_path_parameters.yaml').write_text(common.read_text())
    expansion = profile.parents[1]/'campus_agv_drivable_area_expansion.param.yaml'
    (args.output_dir/'effective_expansion_parameters.yaml').write_text(expansion.read_text())
    children = {}
    log_path = args.output_dir/'launch.log'
    with log_path.open('w') as log, (args.output_dir/'play.log').open('w') as play_log, (args.output_dir/'record.log').open('w') as record_log, (args.output_dir/'configuration.log').open('w') as configuration_log:
        launch = subprocess.Popen(launch_command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        player = recorder = configuration = None
        try:
            until = time.monotonic()+150
            ready = False
            while launch.poll() is None and time.monotonic()<until:
                track(launch,children)
                if "Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" in log_path.read_text(errors='replace'):
                    ready = True
                    break
                time.sleep(0.5)
            if not ready:
                raise RuntimeError('logging simulator not ready')
            recorder = subprocess.Popen(record_command,cwd=ROOT,env=env,stdout=record_log,stderr=subprocess.STDOUT,start_new_session=True)
            configuration = subprocess.Popen(configuration_command,cwd=ROOT,env=env,stdout=configuration_log,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(2)
            player = subprocess.Popen(play_command,cwd=ROOT,env=env,stdout=play_log,stderr=subprocess.STDOUT,start_new_session=True)
            until = time.monotonic()+360
            while player.poll() is None and time.monotonic()<until:
                track(player,children)
                time.sleep(0.5)
            text = log_path.read_text(errors='replace')
            result = {'player_finished':player.poll()==0,
                      'safe_candidates':len(re.findall(r'phase=\w+ safe=1',text)),
                      'committed':bool(re.search(r'phase=(borrowing|passing|returning) safe=1',text)),
                      'reasons':sorted(set(re.findall(r'reason=([^\s]+)',text))),
                      'candidate_speeds':sorted(set(re.findall(r'candidate_speed=([^\s]+)',text)))}
            (args.output_dir/'verdict.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(result,indent=2))
            return 0 if result['committed'] else 1
        finally:
            for process in [player,configuration,recorder]:
                if process:
                    cleanup(process,{})
            cleanup(launch,children)


if __name__ == '__main__':
    raise SystemExit(main())
