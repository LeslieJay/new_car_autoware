#!/usr/bin/env python3
"""ROS-independent map, geometry and verdict helpers for the dual-module suite."""
from __future__ import annotations

import hashlib
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def tags(element):
    return {t.get('k'): t.get('v') for t in element.findall('tag')}


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def interpolate(points, distance):
    if distance < 0:
        raise ValueError('negative map arclength')
    for a, b in zip(points, points[1:]):
        length = math.dist(a[:2], b[:2])
        if distance <= length and length > 0:
            t = distance / length
            return dict(x=a[0]+t*(b[0]-a[0]), y=a[1]+t*(b[1]-a[1]),
                        z=a[2]+t*(b[2]-a[2]), yaw=math.atan2(b[1]-a[1], b[0]-a[0]))
        distance -= length
    raise ValueError('requested position extends beyond map centerline')


def load_map(path):
    root = ET.parse(path).getroot()
    nodes = {}
    for node in root.findall('node'):
        t = tags(node)
        nodes[node.get('id')] = (float(t['local_x']), float(t['local_y']), float(t.get('ele', 0)))
    ways = {w.get('id'): dict(points=[nodes[n.get('ref')] for n in w.findall('nd')],
                             tags=tags(w)) for w in root.findall('way')}
    lanes = {}
    for relation in root.findall('relation'):
        t = tags(relation)
        if t.get('type') != 'lanelet' or t.get('subtype') != 'road':
            continue
        m = {m.get('role'): m.get('ref') for m in relation.findall('member')}
        left, right = ways[m['left']]['points'], ways[m['right']]['points']
        if 'centerline' not in m:
            # Do not invent a routable reference from unequally sampled boundaries.
            center = None
        else:
            center = ways[m['centerline']]['points']
        lanes[relation.get('id')] = dict(tags=t, members=m, left=left, right=right,
            polygon=left + list(reversed(right)), center=center,
            length=sum(math.dist(a[:2], b[:2]) for a,b in zip(center or [], (center or [])[1:])))
    pairs = []
    for aid, a in lanes.items():
        for bid, b in lanes.items():
            if aid >= bid or not a['center'] or not b['center']:
                continue
            shared = set(a['members'].get(k) for k in ('left','right')) & set(b['members'].get(k) for k in ('left','right'))
            if not shared:
                continue
            av = [a['center'][-1][i]-a['center'][0][i] for i in (0,1)]
            bv = [b['center'][-1][i]-b['center'][0][i] for i in (0,1)]
            pairs.append(dict(lanes=[aid,bid], direction='same' if sum(x*y for x,y in zip(av,bv)) > 0 else 'opposite',
                              boundaries={w:ways[w]['tags'] for w in shared}))
    return dict(path=str(path), sha256=file_hash(path), lanes=lanes, shared_boundary_pairs=pairs)


def footprint(x, y, yaw, rear, front, half_width):
    c,s = math.cos(yaw), math.sin(yaw)
    return [(x+u*c-v*s,y+u*s+v*c) for u,v in
            [(-rear,-half_width),(front,-half_width),(front,half_width),(-rear,half_width)]]


def point_segment(p, a, b):
    dx,dy = b[0]-a[0],b[1]-a[1]
    den = dx*dx+dy*dy
    t = max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def inside(p, polygon):
    result = False
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if point_segment(p,a,b) < 1e-6:
            return True
        if (a[1]>p[1]) != (b[1]>p[1]) and p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:
            result = not result
    return result


def simplify(points, tolerance=0.001):
    """Reduce map sampling density with a bounded 1 mm geometric error."""
    if len(points)<3:
        return points
    index,distance=max(enumerate((point_segment(p,points[0],points[-1]) for p in points[1:-1]),1),key=lambda x:x[1])
    if distance<=tolerance:
        return [points[0],points[-1]]
    return simplify(points[:index+1],tolerance)[:-1]+simplify(points[index:],tolerance)


def rectangle_distance(a,b):
    separated = False
    for poly in (a,b):
        for u,v in zip(poly,poly[1:]+poly[:1]):
            dx,dy = v[0]-u[0],v[1]-u[1]
            pa,pb = ([-dy*x+dx*y for x,y in p] for p in (a,b))
            separated |= max(pa)<min(pb) or max(pb)<min(pa)
    if not separated:
        return 0.0
    return min(point_segment(p,u,v) for first,second in ((a,b),(b,a)) for p in first
               for u,v in zip(second,second[1:]+second[:1]))


def manager_state(text):
    result = {}
    for kind in ('registered','approved','candidate'):
        match = re.search(rf'{kind} modules\s*:\s*([^\n]*)',text)
        result[kind] = re.findall(r'\[([a-z_]+)(?:\(([A-Z_]+)\))?\]',match[1]) if match else []
    return result


def verdict(data):
    """Unknown evidence stays inconclusive; an observed safety failure always wins."""
    if data.get('collision') or data.get('boundary_violations',0) or data.get('validator_non_latency_invalid',data.get('validator_invalid',0)):
        return 'ANOMALY'
    if data.get('validator_invalid',0):
        # Keep timing failures invalid. They must not establish a functional
        # capability boundary on a busy development host, or be counted as PASS.
        return 'ENV_INVALID'
    if data.get('environment_errors'):
        return 'ENV_INVALID'
    if not data.get('bag_valid') or not data.get('manager_samples') or not data.get('geometry_samples'):
        return 'INCONCLUSIVE'
    if data.get('expected_objects',0) and not data.get('objects_verified'):
        return 'ENV_INVALID'
    if data.get('arrived') and data.get('passed_all'):
        if data.get('opposite_entered') and not data.get('returned'):
            return 'ANOMALY'
        return 'PASS'
    if data.get('released'):
        return 'ANOMALY' if data.get('control_ready') else 'ENV_INVALID'
    if data.get('stopped_safely') and data.get('refusal_evidence'):
        return 'SAFE_STOP'
    return 'INCONCLUSIVE'
