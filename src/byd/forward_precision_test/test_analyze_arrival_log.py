#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).with_name('analyze_arrival_log.py')
SPEC = importlib.util.spec_from_file_location('analyze_arrival_log', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ArrivalLogAnalyzerTest(unittest.TestCase):

    def test_parses_arrived_and_filters_moving_sample(self):
        content = '\n'.join((
            '[INFO] Goal arrived: goal=(10.0000, 20.0000) pose=(10.0300, 19.9600) '
            'longitudinal=+0.0300m lateral=-0.0400m distance=0.0500m '
            'yaw=+1.20deg speed=0.0100m/s',
            '[INFO] Goal arrived: goal=(0.0000, 0.0000) pose=(1.0000, 0.0000) '
            'longitudinal=+1.0000m lateral=+0.0000m distance=1.0000m '
            'yaw=+0.00deg speed=0.2000m/s',
        ))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'arrival.log'
            path.write_text(content, encoding='utf-8')
            samples, rejected = MODULE.parse_logs([path], 'Goal arrived', 0.05)
        self.assertEqual(len(samples), 1)
        self.assertEqual(rejected, 1)
        self.assertAlmostEqual(samples[0].dx, 0.03)
        self.assertAlmostEqual(samples[0].dy, -0.04)


if __name__ == '__main__':
    unittest.main()
