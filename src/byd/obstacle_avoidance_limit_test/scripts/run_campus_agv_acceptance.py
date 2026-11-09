#!/usr/bin/env python3
"""Run each campus acceptance case in a fresh simulator and retain all evidence."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import psutil

ROOT = Path(__file__).resolve().parents[4]
SCENARIOS = ['baseline','gap5','passing','returning','pedestrian','blocked']


def cleanup(process, descendants):
    for pid, created in descendants.items():
        try:
            child = psutil.Process(pid)
            if child.create_time() == created:
                child.send_signal(signal.SIGINT)
        except psutil.Error:
            pass
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def track(process, descendants):
    try:
        for child in psutil.Process(process.pid).children(recursive=True):
            descendants[child.pid] = child.create_time()
    except psutil.Error:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', action='append', choices=SCENARIOS)
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--domain', type=int, default=174)
    parser.add_argument('--goal-x', type=float, default=-220.0)
    parser.add_argument('--output-root', type=Path, default=ROOT/'log'/datetime.date.today().strftime('%Y%m%d')/('campus_acceptance_'+datetime.datetime.now().strftime('%H%M%S')))
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    args.output_root.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    records = []
    for scenario in args.scenario or SCENARIOS:
        for iteration in range(1,args.repeat+1):
            out = args.output_root/f'{scenario}_{iteration}'
            out.mkdir()
            env['ROS_LOG_DIR'] = str(out/'roslog')
            launch_log = out/'launch.log'
            command = ['ros2','launch','autoware_launch','planning_simulator.launch.xml',
                       'avoidance_by_lane_change_profile:=campus_agv','initial_engage_state:=false',
                       'enable_all_modules_auto_mode:=true','perception/enable_detection_failure:=false',
                       'rviz:=false','launch_static_obstacle_avoidance:=false',
                       'launch_simple_avoidance:=false','launch_simple_lc_avoidance:=false']
            test_command = [sys.executable,str(Path(__file__).with_name('campus_agv_scenario.py')),
                            '--scenario',scenario,'--launch-log',str(launch_log),'--output-dir',str(out),
                            '--goal-x',str(args.goal_x)]
            (out/'commands.json').write_text(json.dumps({'launch':command,'test':test_command,
                                                         'ROS_DOMAIN_ID':args.domain},indent=2))
            descendants = {}
            with launch_log.open('w') as log:
                launch = subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                test = None
                try:
                    ready = False
                    until = time.monotonic()+150
                    while launch.poll() is None and time.monotonic()<until:
                        track(launch,descendants)
                        content = launch_log.read_text(errors='replace')
                        if "Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" in content:
                            ready = True
                            break
                        time.sleep(0.5)
                    if ready:
                        with (out/'test.log').open('w') as test_log:
                            test = subprocess.Popen(test_command,cwd=ROOT,env=env,stdout=test_log,stderr=subprocess.STDOUT,start_new_session=True)
                            until = time.monotonic()+420
                            while test.poll() is None and time.monotonic()<until:
                                track(launch,descendants)
                                track(test,descendants)
                                time.sleep(0.5)
                            if test.poll() is None:
                                cleanup(test,descendants)
                    verdict = json.loads((out/'verdict.json').read_text()) if (out/'verdict.json').exists() else {'passed':False,'reason':'scenario_failed' if ready else 'launch_not_ready'}
                    records.append({'scenario':scenario,'iteration':iteration,'output':str(out),**{k:v for k,v in verdict.items() if k != 'legacy_metrics'}})
                    (args.output_root/'summary.json').write_text(json.dumps(records,indent=2))
                    print(json.dumps(records[-1]),flush=True)
                finally:
                    if test and test.poll() is None:
                        cleanup(test,descendants)
                    cleanup(launch,descendants)
            # Stop on a failure so repeated runs do not conceal the first regression.
            if not records[-1]['passed']:
                return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
