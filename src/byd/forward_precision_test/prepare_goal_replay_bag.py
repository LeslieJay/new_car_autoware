#!/usr/bin/env python3
"""Extract recorded planner/controller inputs and prime held state before a seeked replay."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sqlite3
from zoneinfo import ZoneInfo

import rosbag2_py
import yaml

INPUTS = [
    '/localization/kinematic_state', '/localization/acceleration',
    '/perception/object_recognition/objects',
    '/perception/object_recognition/tracking/objects',
    '/perception/occupancy_grid_map/map', '/perception/obstacle_segmentation/pointcloud',
    '/planning/mission_planning/route', '/planning/route', '/planning/route_state',
    '/planning/mission_planning/state', '/system/operation_mode/state',
    '/vehicle/status/velocity_status', '/vehicle/status/steering_status',
    '/vehicle/status/gear_status', '/vehicle/status/control_mode',
    '/vehicle/status/actuation_status', '/tf', '/tf_static',
]
HELD = {'/planning/mission_planning/route','/planning/route','/planning/route_state',
        '/planning/mission_planning/state','/system/operation_mode/state',
        '/vehicle/status/gear_status','/vehicle/status/control_mode','/tf_static'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--start',default='2026-11-18T15:23:00')
    parser.add_argument('--end',default='2026-11-18T15:27:23')
    args=parser.parse_args()
    tz=ZoneInfo('Asia/Shanghai')
    begin=int(datetime.fromisoformat(args.start).replace(tzinfo=tz).timestamp()*1e9)
    end=int(datetime.fromisoformat(args.end).replace(tzinfo=tz).timestamp()*1e9)
    db=args.source if args.source.is_file() else next(args.source.glob('*.db3'))
    con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
    metadata_path=db.parent/'metadata.yaml'
    counts_in={}
    if metadata_path.exists():
        meta=yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
        counts_in={t['topic_metadata']['name']:t['message_count'] for t in meta['topics_with_message_count']}
    topics={r[0]:r for r in con.execute('select id,name,type,serialization_format,offered_qos_profiles from topics') if r[1] in INPUTS and counts_in.get(r[1],1)>0}
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    counts={r[1]:0 for r in topics.values()};primed=[]
    for tid,name,typ,fmt,qos in topics.values():
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format=fmt,offered_qos_profiles=qos))
    for tid,name,typ,fmt,qos in topics.values():
        if name not in HELD: continue
        order='asc' if name=='/tf_static' else 'desc'
        row=con.execute('select timestamp,data from messages where topic_id=? and timestamp<? order by timestamp '+order+' limit 1',(tid,begin)).fetchone()
        if row:
            writer.write(name,row[1],begin);counts[name]+=1;primed.append(name)
    query='select topic_id,timestamp,data from messages where timestamp between ? and ? and topic_id in ('+','.join(map(str,topics))+') order by timestamp'
    for tid,t,payload in con.execute(query,(begin,end)):
        name=topics[tid][1];writer.write(name,payload,t);counts[name]+=1
    del writer
    manifest=dict(source=str(db),start_ns=begin,end_ns=end,primed_topics=primed,topic_counts=counts,
                  excludes_recorded_planning_and_control_outputs=True)
    (args.output.parent/(args.output.name+'_manifest.json')).write_text(json.dumps(manifest,indent=2))
    print(json.dumps(manifest,indent=2))

if __name__=='__main__':main()
