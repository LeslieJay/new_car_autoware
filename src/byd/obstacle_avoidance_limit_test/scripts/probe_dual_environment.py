#!/usr/bin/env python3
"""Read-only, short pass/fail probe; never opens an active rosbag database."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import OperationModeState


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    rclpy.init()
    node=Node('dual_environment_probe')
    streams={k:dict(samples=0,ages=[],speed=None,position=None) for k in ('reliable','best_effort')}
    mode={}
    def odometry(name,msg):
        stream=streams[name]
        stream['samples']+=1
        stream['ages'].append(node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)
        stream['speed']=msg.twist.twist.linear.x
        p=msg.pose.pose.position
        stream['position']=[p.x,p.y]
    for name,policy in [('reliable',ReliabilityPolicy.RELIABLE),('best_effort',ReliabilityPolicy.BEST_EFFORT)]:
        node.create_subscription(Odometry,'/localization/kinematic_state',lambda m,k=name:odometry(k,m),
                                 QoSProfile(depth=10,reliability=policy))
    def state(msg):
        mode.update(mode=msg.mode,control_enabled=msg.is_autoware_control_enabled,transition=msg.is_in_transition)
    node.create_subscription(OperationModeState,'/api/operation_mode/state',state,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline=time.monotonic()+args.seconds
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
        for value in streams.values():
            ages=value.pop('ages')
            value['max_age']=max(ages) if ages else None
        result=dict(streams=streams,mode=mode,publishers=[p.node_namespace+'/'+p.node_name
            for p in node.get_publishers_info_by_topic('/localization/kinematic_state')])
        result['passed']=all(v['samples']>=5*args.seconds and v['max_age']<0.5 for v in streams.values())
        result['criterion']='Both QoS subscribers receive >=5 Hz odometry, maximum age <0.5 s'
        payload=json.dumps(result,indent=2)
        print(payload,flush=True)
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(payload)
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
