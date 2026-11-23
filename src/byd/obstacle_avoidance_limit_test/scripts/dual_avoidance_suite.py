#!/usr/bin/env python3
"""Launch isolated A/B/C cases, preserve evidence, and render a resumable report."""
from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime
from zoneinfo import ZoneInfo

import yaml
from dual_avoidance_common import file_hash, interpolate, load_map

ROOT = Path(os.environ.get('DUAL_AVOIDANCE_ROOT',str(Path(__file__).resolve().parents[4])))
BASE = Path(__file__).resolve().parents[1]
BPP = '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'


def write_json(path, value):
    temporary=path.with_name(path.name+f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False))
    temporary.replace(path)


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT,15),(signal.SIGTERM,5),(signal.SIGKILL,5)):
        try:
            os.killpg(process.pid,sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def prepare_case(config, case, map_data):
    case = dict(case)
    if case.get('applicability') == 'unsupported':
        return dict(case, skip=case['reason'])
    if case.get('requires') == 'same_direction':
        pairs = [p for p in map_data['shared_boundary_pairs'] if p['direction']=='same']
        # A shared ID is only a candidate, not proof of routable lane-change permission.
        return dict(case, skip='No same-direction shared-boundary pair in map' if not pairs else
                    'Topology exists, but no validated scenario placement/permission; coverage gap')
    route = case.get('route',config['route'])
    lane = map_data['lanes'][route['lane']]
    goal_lane = map_data['lanes'][route.get('goal_lane',route['lane'])]
    ego = interpolate(lane['center'],route['start_s'])
    goal = interpolate(goal_lane['center'],route['goal_s'])
    length = case.get('route_length_bound',route['goal_s']-route['start_s'])
    if length <= 0:
        raise ValueError(f"invalid route length: {case['name']}")
    speed = float(case.get('speed',config['speed']))
    case.update(ego=ego,goal=goal,lane=route['lane'],speed=speed,route=route,
                timeout=max(120,length/speed*3+30),route_length_bound=length)
    center = interpolate(lane['center'],route['start_s'])
    # Map width at the starting projection, not the old hard-coded 4 m width.
    from dual_avoidance_common import point_segment
    center_xy = (center['x'],center['y'])
    case['lane_width'] = sum(min(point_segment(center_xy,a,b) for a,b in zip(lane[k],lane[k][1:]))
                             for k in ('left','right'))
    objects = []
    specs = [] if case.get('kind')=='empty' else [case]
    if case.get('secondary'):
        specs.append(case['secondary'])
    for index,spec in enumerate(specs):
        width,length_obj = spec.get('width',1.5),spec.get('length',2.0)
        distance = spec.get('distance',case.get('distance',35))
        s = route['start_s']+distance
        if s < 0 or s >= lane['length']:
            return dict(case,skip='Object placement exceeds selected lane centerline')
        pose = interpolate(lane['center'],s)
        lateral = spec.get('lateral')
        if lateral is None:
            lateral = case['lane_width']/2+width/2-spec.get('intrusion',1.0)
            lateral *= -1 if spec.get('side','right')=='right' else 1
        pose['x'] -= math.sin(pose['yaw'])*lateral
        pose['y'] += math.cos(pose['yaw'])*lateral
        if spec.get('reverse'):
            pose['yaw'] += math.pi
        objects.append(dict(pose=pose,width=width,length=length_obj,height=1.5,
            label=spec.get('label','car'),speed=spec.get('object_speed',0) if index==0 else spec.get('speed',0),
            delay_until_running=case.get('event')=='add_running' and index==1,
            initially_ahead=distance>0, lateral=lateral))
    case['objects'] = objects
    return case


def expand_cases(config, selected, sweep, start=0.0, stop=2.0):
    cases = config['cases']
    if selected:
        names = set(selected.split(','))
        unknown = names-{c['name'] for c in cases}
        if unknown:
            raise ValueError(f'Unknown cases: {sorted(unknown)}')
        cases = [c for c in cases if c['name'] in names]
    if sweep:
        # Boundary sweeps are explicit extra runs; never silently replace representative cases.
        step = 0.2 if sweep=='coarse' else 0.05
        cases = [dict(name=f'{side}_{start+i*step:.2f}',distance=40.0,intrusion=round(start+i*step,3),side=side)
                 for side in ('left','right') for i in range(math.floor((stop-start)/step+1e-7)+1)]
    return cases


def report(out):
    manifest = json.loads((out/'manifest.json').read_text())
    rows = []
    for entry in manifest['runs']:
        path = out/entry['id']/'verdict.json'
        value = json.loads(path.read_text()) if path.exists() else {'result':'PENDING'}
        rows.append(dict(id=entry['id'],scenario=entry['case']['name'],group=entry['group'],
                         repeat=entry['repeat'],result=value['result'],
                         speed=entry['case'].get('speed'),intrusion=entry['case'].get('intrusion'),
                         distance=entry['case'].get('distance'),injection_speed=value.get('actual_injection_speed'),
                         approved=','.join(value.get('executed_modules',[])),
                         arrived=value.get('arrived'),min_clearance=value.get('min_clearance'),
                         process_failures=','.join(f"{f['process']}:{f['exit_code']}" for f in value.get('process_failures',[])),
                         reason='; '.join(value.get('environment_errors',[])),
                         evidence=str(path.relative_to(out))))
    with (out/'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(rows[0]) if rows else ['id'])
        writer.writeheader(); writer.writerows(rows)
    from collections import Counter
    counts = Counter(r['result'] for r in rows)
    state_path=out/'runner_state.json'
    state=json.loads(state_path.read_text()) if state_path.exists() else {'phase':'PREPARED'}
    lines = ['# 双模块绕障测试报告','',
        '本报告只依据已完成用例；PENDING 不代表测试通过。候选路径、RTC 请求和 approved RUNNING 分别统计。',
        '',f"地图：`{manifest['map']['path']}`；SHA256：`{manifest['map']['sha256']}`。",'',
        '批次状态：`'+state['phase']+'`；'+state.get('message',''),'',
        'A=仅 static；B=仅 avoidance_by_lane_change；C=双开。保留其他模块和安全停车功能。','',
        '结果统计：'+', '.join(f'{k}={v}' for k,v in sorted(counts.items())), '',
        'PASS=完整到达且有安全证据；SAFE_STOP=安全停车且有拒绝/停车原因，不能等同于已验证恢复；',
        'INCONCLUSIVE=证据不足或未完成；ENV_INVALID=环境/输入无效；ANOMALY=安全失败或解除限制后未恢复。','',
        '| 用例 | 组 | 重复 | 结果 | 获准运行模块 | 证据 |',
        '|---|---|---|---|---|---|']
    for r in rows:
        lines.append(f"| {r['scenario']} | {r['group']} | {r['repeat']} | {r['result']} | {r['approved']} | [verdict]({r['evidence']}) |")
    lines += ['', '## A/B/C 行为对照','',
              '| 场景 | A | B | C | 对照状态 |','|---|---|---|---|---|']
    for scenario in dict.fromkeys(r['scenario'] for r in rows):
        by_group={g:[r for r in rows if r['scenario']==scenario and r['group']==g] for g in ('A','B','C')}
        cells=[' / '.join(f"{r['result']} ({r['approved'] or '无获准模块'})" for r in by_group[g]) or '未计划'
               for g in ('A','B','C')]
        complete=all(by_group[g] and all(r['result']!='PENDING' for r in by_group[g]) for g in by_group)
        comparison='数据未齐，不作归因'
        if complete:
            signatures=[{(r['result'],r['approved']) for r in by_group[g]} for g in ('A','B','C')]
            comparison='双开行为与两组单开均有差异' if signatures[2] not in signatures[:2] else '双开行为与至少一组单开一致'
            if any(len(s)>1 for s in signatures):
                comparison+='；重复结果不一致，需复现'
        lines.append('| '+' | '.join([scenario,*cells,comparison])+' |')
    lines += ['', '## 覆盖缺口','']
    for case in manifest['skipped']:
        lines.append(f"- {case['name']}: {case['skip']}")
    lines+=['','## 节点退出记录','',
        '固定路线测试不调用手动换道路由接口；该接口崩溃单列，不代表已验证其可用。其他节点退出记为环境无效。','']
    for row in rows:
        if row['process_failures']:
            lines.append(f"- {row['id']}: {row['process_failures']}; [证据]({row['evidence']})")
    lines+=['','## 环境无效重试记录','']
    attempts=sorted((out/'retry_attempts').glob('**/verdict.json'))
    for path in attempts:
        previous=json.loads(path.read_text())
        lines.append(f"- [{path.parent.relative_to(out)}]({path.relative_to(out)}): {previous['result']}; "+
                     '; '.join(previous.get('environment_errors',[])))
    if not attempts:
        lines.append('暂无。每个用例最多重试两次；原始无效结果与 bag 保留，不计为通过。')
    lines += ['', '## 结论边界','',
        '- 执行归属来自 planner manager 的 approved RUNNING；不依据跨线或横向偏移推断。',
        '- 未发布内部请求状态时，记录为不可观测，不能断言模块没有请求；候选和调度信息保存在时间线。',
        '- 净距以实际里程计车身与注入物体运动模型计算；感知 UUID、预测轨迹另存 bag。',
        '- 道路外边界基于地图采样检查；具体安全裕度、允许区域仍需结合运行时参数与 bag 审核。',
        '- 规划验证器失效、bag 丢失或关键状态缺失不会被判为 PASS。','']
    (out/'report.md').write_text('\n'.join(lines))
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=BASE/'config/dual_avoidance.yaml')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--cases',help='Comma-separated names; default all cases')
    parser.add_argument('--groups',default=None)
    parser.add_argument('--repeats',type=int)
    parser.add_argument('--domain',type=int,default=174)
    parser.add_argument('--rmw',choices=['rmw_cyclonedds_cpp','rmw_fastrtps_cpp'],
                        default=os.environ.get('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'))
    parser.add_argument('--environment-retries',type=int,choices=[0,1,2],default=2)
    parser.add_argument('--sweep',choices=['coarse','fine'])
    parser.add_argument('--sweep-start',type=float,default=0.0)
    parser.add_argument('--sweep-stop',type=float,default=2.0)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--report-only',action='store_true')
    parser.add_argument('--max-load',type=float,default=1.5*(os.cpu_count() or 1),
                        help='Wait before launching when 1-minute system load exceeds this value')
    args = parser.parse_args()
    out = (args.output or ROOT/'log'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d/dual_avoidance_%H%M%S')).resolve()
    if args.report_only:
        print(dict(report(out))); return
    config = yaml.safe_load(args.config.read_text())
    if not 1 <= args.domain <= 232:
        parser.error('domain must be between 1 and 232')
    groups = args.groups.split(',') if args.groups else config['groups']
    repeats = args.repeats if args.repeats is not None else config['repeats']
    if any(g not in ('A','B','C') for g in groups) or repeats<1 or args.environment_retries<0:
        parser.error('groups must be A,B,C and repeats must be positive')
    out.mkdir(parents=True,exist_ok=True)
    if (out/'manifest.json').exists() and not args.resume:
        parser.error('output exists; use --resume or choose another output')
    if args.resume:
        manifest = json.loads((out/'manifest.json').read_text())
        config=manifest['config']
        args.rmw=manifest.get('rmw_implementation',args.rmw)
        if file_hash(manifest['map']['path']) != manifest['map']['sha256']:
            raise RuntimeError('Map changed since manifest; start a new suite')
    else:
        map_data = load_map(Path(config['map_dir'])/config['map_file'])
        write_json(out/'map_inventory.json',map_data)
        projector = yaml.safe_load((Path(config['map_dir'])/'map_projector_info.yaml').read_text())
        if projector['projector_type'] != 'LocalCartesian':
            raise RuntimeError('Only the validated LocalCartesian map is supported')
        if args.sweep_stop<args.sweep_start:
            parser.error('sweep-stop must be >= sweep-start')
        cases = [prepare_case(config,c,map_data) for c in expand_cases(config,args.cases,args.sweep,args.sweep_start,args.sweep_stop)]
        runs = [dict(id=f"{c['name']}/{g}/r{r}",case=c,group=g,repeat=r)
                for c in cases if 'skip' not in c for r in range(1,repeats+1) for g in groups]
        manifest = dict(config=config,map={k:map_data[k] for k in ('path','sha256')},projector=projector,
                        runs=runs,skipped=[c for c in cases if 'skip' in c],rmw_implementation=args.rmw)
        write_json(out/'manifest.json',manifest)
    print(f"Evidence directory: {out}; runs={len(manifest['runs'])}",flush=True)
    report(out)
    if args.prepare_only:
        return
    import shutil
    runtime=out/'runtime'
    if not runtime.exists():
        shutil.copytree(BASE/'scripts',runtime/'scripts',ignore=shutil.ignore_patterns('__pycache__'))
        shutil.copytree(BASE/'config',runtime/'config')
        write_json(out/'harness_hashes.json',{str(p.relative_to(runtime)):file_hash(p)
                   for p in runtime.rglob('*') if p.is_file()})
    artifact_path=out/'artifact_hashes.json'
    if not artifact_path.exists():
        from ament_index_python.packages import get_package_share_directory
        launch_share=Path(get_package_share_directory('autoware_launch'))
        artifacts=list((launch_share/'config').rglob('*.yaml'))+list((launch_share/'launch').rglob('*.xml'))
        artifacts += [Path(manifest['map']['path']),Path(config['map_dir'])/'map_projector_info.yaml']
        for package in ('autoware_behavior_path_avoidance_by_lane_change_module',
                        'autoware_behavior_path_static_obstacle_avoidance_module','autoware_behavior_path_planner'):
            artifacts += list((ROOT/'install'/package/'lib').glob('*.so'))
        write_json(artifact_path,{str(p):file_hash(p) for p in artifacts})
    artifact_hashes=json.loads(artifact_path.read_text())
    # Lock this private domain. Never kill a user's ROS session to acquire it.
    lock = open(out/'.suite.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    env = dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',DUAL_AVOIDANCE_ROOT=str(ROOT),
               RMW_IMPLEMENTATION=args.rmw,RCUTILS_COLORIZED_OUTPUT='0')
    if args.rmw=='rmw_cyclonedds_cpp' and not env.get('CYCLONEDDS_URI'):
        # Localhost-only discovery has a small default auto-index search range.
        # The full simulator has dozens of OS processes/participants. Keep the
        # enlarged range private to this suite and do not edit host DDS settings.
        profile=out/'cyclonedds.xml'
        if not profile.exists():
            profile.write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any">'
                '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
                '<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>'
                '<Peers><Peer address="127.0.0.1"/></Peers></Discovery>'
                '</Domain></CycloneDDS>\n')
        env['CYCLONEDDS_URI']=str(profile)
    check = subprocess.run(['ros2','node','list','--no-daemon'],env=env,capture_output=True,text=True,timeout=20)
    if check.returncode or check.stdout.strip():
        raise RuntimeError(f'Domain {args.domain} unavailable: {check.stdout} {check.stderr}')
    launch = driver = None
    def interrupted(_signum,_frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    retries={}
    pending=list(enumerate(manifest['runs'],1))
    try:
        while pending:
            index,entry=pending.pop(0)
            directory = out/entry['id']
            if (directory/'verdict.json').exists():
                continue
            while os.getloadavg()[0]>args.max_load:
                message=f"Next {entry['id']}; load={os.getloadavg()[0]:.1f}, limit={args.max_load:.1f}, cores={os.cpu_count()}"
                write_json(out/'runner_state.json',dict(phase='WAITING_FOR_CAPACITY',message=message,pid=os.getpid()))
                report(out)
                print('Waiting for capacity: '+message,flush=True)
                time.sleep(30)
            write_json(out/'runner_state.json',dict(phase='RUNNING',message=entry['id'],pid=os.getpid()))
            report(out)
            changed=[path for path,digest in artifact_hashes.items() if not Path(path).exists() or file_hash(path)!=digest]
            if changed:
                raise RuntimeError(f'Configuration/map/binaries changed during suite: {changed}')
            if shutil.disk_usage(out).free < 10*1024**3:
                raise RuntimeError('Less than 10 GiB free; refusing to truncate test evidence')
            if directory.exists():
                archive=out/'interrupted'/f"{entry['id']}_{int(time.time())}"
                archive.parent.mkdir(parents=True,exist_ok=True)
                directory.rename(archive)
            directory.mkdir(parents=True,exist_ok=True)
            case = entry['case']
            domain=1+((args.domain-1+index-1+37*retries.get(entry['id'],0))%220)
            domain_lock=open(f'/tmp/dual_avoidance_domain_{domain}.lock','w')
            fcntl.flock(domain_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            write_json(directory/'case.json',case)
            case_env = dict(env,ROS_DOMAIN_ID=str(domain),ROS_LOG_DIR=str(directory/'roslog'))
            existing=subprocess.run(['ros2','node','list','--no-daemon'],env=case_env,capture_output=True,text=True,timeout=20)
            if existing.returncode or existing.stdout.strip():
                raise RuntimeError(f'Refusing to use occupied domain {domain}: {existing.stdout} {existing.stderr}')
            command = ['ros2','launch','autoware_launch','planning_simulator.launch.xml',
                f"map_path:={config['map_dir']}",f"lanelet2_map_file:={config['map_file']}",
                'pointcloud_map_file:=pointcloud_map.pcd',f"vehicle_model:={config['vehicle_model']}",
                f"sensor_model:={config['sensor_model']}",'launch_didrive_perception:=false',
                'scenario_simulation:=false','rviz:=false','initial_engage_state:=true',
                f"launch_static_obstacle_avoidance:={'false' if entry['group']=='B' else 'true'}",
                f"launch_avoidance_by_lane_change_module:={'false' if entry['group']=='A' else 'true'}"]
            write_json(directory/'launch.json',dict(command=command,domain=domain,
                rmw_implementation=args.rmw,
                cyclone_profile=env.get('CYCLONEDDS_URI'),
                system_load=os.getloadavg(),cpu_count=os.cpu_count(),
                source_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                source_diff=subprocess.check_output(['git','diff'],cwd=ROOT,text=True)))
            print(f"[{index}/{len(manifest['runs'])}] {entry['id']}",flush=True)
            with (directory/'launch.log').open('w') as log, (directory/'driver.log').open('w') as driver_log:
                launch = subprocess.Popen(command,cwd=ROOT,env=case_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                driver_command = [sys.executable,str(runtime/'scripts/dual_avoidance_scenario.py'),
                    '--case',str(directory/'case.json'),'--output',str(directory),
                    '--map-inventory',str(out/'map_inventory.json'),'--group',entry['group'],
                    '--startup-timeout',str(config['startup_timeout'])]
                driver = subprocess.Popen(driver_command,cwd=ROOT,env=case_env,stdout=driver_log,
                                          stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    code = driver.wait(timeout=case['timeout']+config['startup_timeout']+180)
                    if not (directory/'verdict.json').exists():
                        write_json(directory/'verdict.json',dict(result='ENV_INVALID',
                            environment_errors=[f'Driver exited {code}; inspect driver.log']))
                except subprocess.TimeoutExpired:
                    write_json(directory/'verdict.json',dict(result='ENV_INVALID',environment_errors=['Driver process timeout']))
                finally:
                    stop_process(driver); stop_process(launch)
                    driver = launch = None
            report(out)
            domain_lock.close()
            result=json.loads((directory/'verdict.json').read_text())['result']
            if result=='ENV_INVALID' and retries.get(entry['id'],0)<args.environment_retries:
                retries[entry['id']]=retries.get(entry['id'],0)+1
                archive=out/'retry_attempts'/entry['id']/f"attempt_{retries[entry['id']]}_{int(time.time())}"
                archive.parent.mkdir(parents=True,exist_ok=True)
                directory.rename(archive)
                pending.insert(0,(index,entry))
                print(f"Environment invalid; evidence archived at {archive}; retry after capacity check.",flush=True)
                continue
            if case['name']=='empty' and result!='PASS':
                write_json(out/'runner_state.json',dict(phase='BASELINE_FAILED',message=entry['id']))
                print('Baseline did not pass; suite stopped before obstacle cases. Inspect evidence.',flush=True)
                break
        else:
            write_json(out/'runner_state.json',dict(phase='FINISHED',message='All planned runs have a verdict; inspect outcome counts.'))
    except KeyboardInterrupt:
        write_json(out/'runner_state.json',dict(phase='INTERRUPTED',message='Owned simulator and recorder stopped; partial evidence preserved.'))
        raise
    except Exception as error:
        write_json(out/'runner_state.json',dict(phase='ERROR',message=str(error)))
        raise
    finally:
        stop_process(driver); stop_process(launch)
        report(out)


if __name__=='__main__':
    main()
