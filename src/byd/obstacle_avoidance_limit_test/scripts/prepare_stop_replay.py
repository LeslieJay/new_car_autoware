#!/usr/bin/env python3
"""Create an input-only replay crop with latched state before the incident.

Source SQLite is opened read-only. No recorded planning or control output is replayed.
"""
import argparse
import datetime as dt
import json
from pathlib import Path
import sqlite3
import rosbag2_py

INPUTS = {
 '/localization/kinematic_state', '/localization/acceleration',
 '/perception/object_recognition/objects', '/perception/occupancy_grid_map/map',
 '/perception/obstacle_segmentation/pointcloud', '/vehicle/status/steering_status',
 '/vehicle/status/gear_status', '/vehicle/status/control_mode',
 '/vehicle/status/velocity_status', '/system/operation_mode/state',
 '/planning/mission_planning/route', '/planning/mission_planning/state',
 '/tf', '/tf_static',
}
LATCHED = {'/tf_static','/planning/mission_planning/route',
           '/planning/mission_planning/state','/system/operation_mode/state'}

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--start',default='2026-11-20T16:46:40+08:00')
 p.add_argument('--end',default='2026-11-20T16:48:10+08:00')
 a=p.parse_args();start=int(dt.datetime.fromisoformat(a.start).timestamp()*1e9);end=int(dt.datetime.fromisoformat(a.end).timestamp()*1e9)
 c=sqlite3.connect(f'file:{a.source}?mode=ro',uri=True)
 topics={r[0]:r for r in c.execute('select id,name,type,serialization_format,offered_qos_profiles from topics') if r[1] in INPUTS}
 a.output.parent.mkdir(parents=True,exist_ok=True)
 w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(a.output),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
 for _,n,t,s,q in topics.values():w.create_topic(rosbag2_py.TopicMetadata(name=n,type=t,serialization_format=s,offered_qos_profiles=q))
 counts={r[1]:0 for r in topics.values()};seed={}
 for i,n,*_ in topics.values():
  if n not in LATCHED:continue
  row=c.execute('select timestamp,data from messages where timestamp<=? and topic_id=? order by timestamp desc limit 1',(start,i)).fetchone()
  if row:w.write(n,row[1],start-1000000);counts[n]+=1;seed[n]=row[0]
  print('seed',n,seed.get(n),flush=True)
 ids=','.join(map(str,topics))
 for t,i,b in c.execute(f'select timestamp,topic_id,data from messages where timestamp between ? and ? and topic_id in ({ids}) order by timestamp',(start,end)):
  n=topics[i][1];w.write(n,b,t);counts[n]+=1
 del w
 manifest={'source':str(a.source),'start':a.start,'end':a.end,'counts':counts,'seed_source_stamps_ns':seed,'recorded_outputs_replayed':False}
 a.output.with_suffix('.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest,indent=2))
 if not all(counts.get(n,0) for n in ['/localization/kinematic_state','/perception/object_recognition/objects','/planning/mission_planning/route']):raise RuntimeError('missing essential input')
if __name__=='__main__':main()
