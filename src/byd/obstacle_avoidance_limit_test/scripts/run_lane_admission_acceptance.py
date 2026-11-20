#!/usr/bin/env python3
"""Run admission cases in isolated planning simulators and retain evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from lane_admission_scenario import CASES
from run_campus_agv_acceptance import ROOT, cleanup, track


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', action='append', choices=sorted(CASES))
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--domain-base', type=int, default=180)
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    args.output_root.mkdir(parents=True, exist_ok=True)
    records = []
    for index, (case, iteration) in enumerate(
        (case, iteration) for case in (args.scenario or sorted(CASES))
        for iteration in range(1, args.repeat + 1)
    ):
        domain = args.domain_base + index
        if domain > 232:
            parser.error('ROS domain exceeds 232')
        out = args.output_root / f'{case}_{iteration}'
        out.mkdir()
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                   RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=str(out/'roslog'),
                   DIRECT_SIMULATOR_POSE='1')
        launch_log = out/'launch.log'
        launch_command = ['ros2', 'launch', 'autoware_launch',
            'planning_simulator.launch.xml', 'avoidance_by_lane_change_profile:=campus_agv',
            'initial_engage_state:=false',
            f'enable_all_modules_auto_mode:={"false" if case in ("move_in_out", "move_deep_in_out") else "true"}',
            'perception/enable_detection_failure:=false', 'rviz:=false',
            'launch_static_obstacle_avoidance:=false', 'launch_simple_avoidance:=false',
            'launch_simple_lc_avoidance:=false']
        test_command = [sys.executable,
            str(Path(__file__).with_name('lane_admission_scenario.py')),
            '--scenario', case, '--launch-log', str(launch_log),
            '--output-dir', str(out)]
        map_path = Path('/home/nvidia/autoware_map/hy_1031/hy_1031_lanelet2_map_with_latlon.osm')
        binary = ROOT/'install/autoware_behavior_path_avoidance_by_lane_change_module/lib/libautoware_behavior_path_avoidance_by_lane_change_module.so'
        (out/'commands.json').write_text(json.dumps({
            'launch': launch_command, 'test': test_command, 'ROS_DOMAIN_ID': domain,
            'map_sha256': sha256(map_path), 'module_sha256': sha256(binary)}, indent=2))
        descendants = {}
        with launch_log.open('w') as log:
            launch = subprocess.Popen(launch_command, cwd=ROOT, env=env, stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            test = None
            try:
                ready = False
                until = time.monotonic() + 150
                while launch.poll() is None and time.monotonic() < until:
                    track(launch, descendants)
                    launch_text = launch_log.read_text(errors='replace')
                    if ("Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" in launch_text and
                            "Loaded node '/adapi/node/operation_mode'" in launch_text):
                        ready = True
                        break
                    time.sleep(0.5)
                if ready:
                    with (out/'test.log').open('w') as test_log:
                        test = subprocess.Popen(test_command, cwd=ROOT, env=env,
                            stdout=test_log, stderr=subprocess.STDOUT, start_new_session=True)
                        until = time.monotonic() + 600
                        while test.poll() is None and time.monotonic() < until:
                            track(launch, descendants)
                            track(test, descendants)
                            time.sleep(0.5)
                        if test.poll() is None:
                            cleanup(test, descendants)
                verdict_path = out/'verdict.json'
                verdict = json.loads(verdict_path.read_text()) if verdict_path.exists() else {
                    'passed': False, 'reason': 'scenario_failed' if ready else 'launch_not_ready'}
                records.append({'scenario': case, 'iteration': iteration, 'domain': domain,
                                'output': str(out), **verdict})
                (args.output_root/'summary.json').write_text(json.dumps(records, indent=2))
                print(json.dumps(records[-1]), flush=True)
            finally:
                if test is not None and test.poll() is None:
                    cleanup(test, descendants)
                cleanup(launch, descendants)
        if not records[-1]['passed']:
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
