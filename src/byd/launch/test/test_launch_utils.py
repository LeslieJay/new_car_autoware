#!/usr/bin/env python3

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

from launch.actions import EmitEvent


SCRIPT_PATH = Path(__file__).parents[1] / "launch_utils.py"
SPEC = importlib.util.spec_from_file_location("launch_utils", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LaunchUtilsTransitionTest(unittest.TestCase):
    def test_soft_readiness_failure_logs_and_continues_without_shutdown(self):
        next_action = object()
        handler = MODULE.on_success(
            [next_action],
            "drivers",
            shutdown_on_failure=False,
        )

        actions = handler(SimpleNamespace(returncode=1), None)

        self.assertIn(next_action, actions)
        self.assertFalse(any(isinstance(action, EmitEvent) for action in actions))


if __name__ == "__main__":
    unittest.main()
