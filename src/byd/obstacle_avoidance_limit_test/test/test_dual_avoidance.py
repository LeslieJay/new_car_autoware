"""Regression checks for false-positive verdicts and map-based placement."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_avoidance_common import footprint, inside, interpolate, manager_state, rectangle_distance, verdict
from dual_avoidance_suite import prepare_case


class DualAvoidanceTest(unittest.TestCase):
    def ready(self,**kwargs):
        value=dict(bag_valid=True,manager_samples=1,geometry_samples=1,arrived=True,passed_all=True)
        value.update(kwargs)
        return value

    def test_candidate_is_not_approved(self):
        parsed=manager_state('registered modules: [static_obstacle_avoidance][avoidance_by_lane_change]\n'
            'approved modules  : [[ [static_obstacle_avoidance(RUNNING)] ]]\n'
            'candidate modules : [[ [avoidance_by_lane_change(WAITING_APPROVAL)] ]]\n')
        self.assertEqual(parsed['approved'],[('static_obstacle_avoidance','RUNNING')])
        self.assertEqual(parsed['candidate'],[('avoidance_by_lane_change','WAITING_APPROVAL')])
        self.assertEqual(len(parsed['registered']),2)

    def test_collision_cannot_be_hidden_by_environment_error(self):
        self.assertEqual(verdict(self.ready(collision=True,environment_errors=['recorder failed'])),'ANOMALY')

    def test_latency_is_retained_and_requires_environment_retry(self):
        self.assertEqual(verdict(self.ready(validator_invalid=1,validator_non_latency_invalid=0)),'ENV_INVALID')
        self.assertEqual(verdict(self.ready(validator_invalid=1,validator_non_latency_invalid=1)),'ANOMALY')

    def test_arrival_is_not_enough(self):
        self.assertEqual(verdict(self.ready(passed_all=False)),'INCONCLUSIVE')
        self.assertEqual(verdict(self.ready(bag_valid=False)),'INCONCLUSIVE')
        self.assertEqual(verdict(self.ready(expected_objects=1,objects_verified=False)),'ENV_INVALID')

    def test_return_required_only_after_opposite_occupancy(self):
        self.assertEqual(verdict(self.ready()),'PASS')
        self.assertEqual(verdict(self.ready(opposite_entered=True,returned=False)),'ANOMALY')
        self.assertEqual(verdict(self.ready(opposite_entered=True,returned=True)),'PASS')

    def test_safely_stopped_requires_reason(self):
        self.assertEqual(verdict(self.ready(arrived=False,stopped_safely=True)),'INCONCLUSIVE')
        self.assertEqual(verdict(self.ready(arrived=False,stopped_safely=True,refusal_evidence=True)),'SAFE_STOP')
        self.assertEqual(verdict(self.ready(arrived=False,released=True,control_ready=True)),'ANOMALY')

    def test_rotated_body_distance(self):
        a=footprint(0,0,math.pi/4,1,1,0.5)
        self.assertEqual(rectangle_distance(a,a),0)
        b=footprint(10,0,0,1,1,0.5)
        self.assertGreater(rectangle_distance(a,b),7)
        self.assertAlmostEqual(rectangle_distance(a,b),rectangle_distance(b,a))

    def test_edge_contact_and_nonconvex_road(self):
        polygon=[(0,0),(4,0),(4,1),(1,1),(1,4),(0,4)]
        self.assertTrue(inside((0,2),polygon))
        self.assertTrue(inside((0.5,3),polygon))
        self.assertFalse(inside((2,2),polygon))

    def test_map_distance_not_extrapolated(self):
        points=[(0,0,1),(10,0,3)]
        self.assertEqual(interpolate(points,5),dict(x=5,y=0,z=2,yaw=0))
        with self.assertRaises(ValueError):
            interpolate(points,11)

    def test_intrusion_measures_inner_edge(self):
        config=dict(speed=1,route=dict(lane='1',start_s=10,goal_s=90))
        lane=dict(center=[(0,0,0),(100,0,0)],left=[(0,2.25,0),(100,2.25,0)],
                  right=[(0,-2.25,0),(100,-2.25,0)],length=100)
        map_data=dict(lanes={'1':lane})
        for side,sign in (('left',1),('right',-1)):
            case=prepare_case(config,dict(name='edge',distance=20,intrusion=0.3,side=side),map_data)
            obj=case['objects'][0]
            inner_edge=sign*obj['pose']['y']-obj['width']/2
            self.assertAlmostEqual(inner_edge,2.25-0.3)


if __name__=='__main__':
    unittest.main()
