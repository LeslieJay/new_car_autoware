#!/usr/bin/env python3
"""Real-process regression for localhost Cyclone participant capacity. Source ROS first."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--profile',default='')
    parser.add_argument('--domain',type=int,default=229)
    parser.add_argument('--count',type=int,default=20)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    env=dict(os.environ,RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_LOCALHOST_ONLY='1',
             ROS_DOMAIN_ID=str(args.domain),CYCLONEDDS_URI=args.profile)
    program=('import rclpy,time; from rclpy.node import Node; rclpy.init(); '
             'n=Node("capacity_probe"); print("READY",flush=True); '
             'time.sleep(6); n.destroy_node(); rclpy.shutdown()')
    processes=[]
    results=[]
    try:
        for _ in range(args.count):
            processes.append(subprocess.Popen([sys.executable,'-c',program],env=env,text=True,
                stdout=subprocess.PIPE,stderr=subprocess.PIPE))
        for process in processes:
            stdout,stderr=process.communicate(timeout=30)
            results.append(dict(code=process.returncode,ready='READY' in stdout,
                                index_exhausted='free participant index' in stderr))
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill(); process.wait()
    value=dict(count=args.count,ready=sum(r['ready'] for r in results),
               index_exhausted=sum(r['index_exhausted'] for r in results),results=results)
    value['passed']=value['ready']==args.count and all(r['code']==0 for r in results)
    print(json.dumps(value,indent=2))
    if args.output:
        args.output.write_text(json.dumps(value,indent=2))
    return 0 if value['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
