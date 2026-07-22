#!/usr/bin/env python3

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).parents[1]


class PedestrianSafetyConfigTest(unittest.TestCase):

    def setUp(self):
        config_path = ROOT / 'config' / 'pedestrian_safety_stop.param.yaml'
        self.params = yaml.safe_load(config_path.read_text())['/**'][
            'ros__parameters'
        ]

    def test_only_unknown_and_pedestrian_are_enabled(self):
        enabled = {
            label
            for label in (
                'unknown',
                'car',
                'truck',
                'bus',
                'trailer',
                'motorcycle',
                'bicycle',
                'pedestrian',
            )
            if self.params[label]['enable_check']
        }
        self.assertEqual(enabled, {'unknown', 'pedestrian'})

    def test_four_sided_zone_and_release_policy(self):
        for label in ('unknown', 'pedestrian'):
            self.assertEqual(
                self.params[label]['surround_check_front_distance'], 2.0
            )
            self.assertEqual(
                self.params[label]['surround_check_side_distance'], 1.0
            )
            self.assertEqual(
                self.params[label]['surround_check_back_distance'], 2.0
            )
        self.assertEqual(self.params['surround_check_hysteresis_distance'], 0.5)
        self.assertEqual(self.params['state_clear_time'], 2.0)

    def test_fail_safe_and_command_gate_are_enabled(self):
        self.assertFalse(self.params['stop_only_when_stopped'])
        self.assertTrue(self.params['fail_safe_on_data_timeout'])
        self.assertEqual(self.params['data_timeout_sec'], 0.5)
        self.assertTrue(self.params['request_command_gate_stop'])
        self.assertEqual(
            self.params['stop_request_source'],
            'byd_pedestrian_safety_stop',
        )


class PedestrianSafetyLaunchTest(unittest.TestCase):

    def test_bringup_variants_gate_auto_engage_on_safety_ready(self):
        for launch_name in ('bringup.launch.py', 'parallel_bringup.launch.py'):
            source = (ROOT / launch_name).read_text()
            self.assertIn('/byd/pedestrian_safety_stop/ready', source)
            self.assertIn('/control/vehicle_cmd_gate/set_stop', source)
            self.assertIn('register_stage_transition', source)


if __name__ == '__main__':
    unittest.main()
