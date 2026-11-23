#!/usr/bin/env python3
"""Check runtime bag evidence for the persistent execution-discontinuity stop.

With --expect-execution-stop, require a reproduced fault. Otherwise reject it.
Missing planner/validator output is always an inconclusive failure.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

FACTOR='/planning/planning_factors/avoidance_by_lane_change'
TRAJECTORY='/planning/trajectory'
VALIDATOR='/planning/planning_validator/validation_status'

def inspect(path):
 dbs=[path] if path.is_file() else sorted(path.glob('*.db3'))
 counts=Counter();reasons=Counter();stamps=[];invalid=Counter();maximum=0.0;trajectories=0
 for db in dbs:
  c=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
  for i,n,t in c.execute('select id,name,type from topics'):
   if n not in [FACTOR,TRAJECTORY,VALIDATOR]:continue
   typ=get_message(t)
   for stamp,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)):
    counts[n]+=1;m=deserialize_message(b,typ)
    if n==FACTOR:
     for f in m.factors:
      reasons[f.detail]+=1
      if f.detail=='execution_discontinuity':stamps.append(stamp/1e9)
    elif n==VALIDATOR:
     for k in m.get_fields_and_field_types():
      if k.startswith('is_valid') and getattr(m,k) is False:invalid[k]+=1
     maximum=max(maximum,abs(m.max_lateral_acc))
    else:trajectories+=bool(m.points)
  c.close()
 duration=max(stamps)-min(stamps) if stamps else 0
 return dict(counts=dict(counts),reasons=dict(reasons),execution_stop_duration_s=duration,
  execution_stop_first_s=min(stamps) if stamps else None,
  reproduced_persistent_stop=len(stamps)>=10 and duration>=2,
  invalid_checks=dict(invalid),max_planned_lateral_acc_mps2=maximum,
  has_runtime_chain=counts[FACTOR]>0 and counts[VALIDATOR]>0 and trajectories>0)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bag',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--expect-execution-stop',action='store_true');a=p.parse_args()
 r=inspect(a.bag);r['passed']=r['has_runtime_chain'] and (r['reproduced_persistent_stop']==a.expect_execution_stop)
 if a.output:a.output.write_text(json.dumps(r,indent=2))
 print(json.dumps(r,indent=2));return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
