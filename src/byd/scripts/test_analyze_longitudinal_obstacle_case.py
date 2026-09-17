import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_longitudinal_obstacle_case import (
    BASE_LINK_TO_VEHICLE_CENTER_M,
    classify_result,
    rectangle_signed_clearance,
)


def evidence(**overrides):
    values = dict(
        bag_valid=True,
        object_count_ok=True,
        add_count_ok=True,
        clear_confirmed=True,
        module_configuration_ok=True,
        has_dynamic_obstacle_stop=False,
        mrm_operation_count=0,
        invalid_trajectory_count=0,
        min_collision_clearance=0.2,
        timeout=False,
        monitor_result="ARRIVED",
        vehicle_passed=True,
        goal_reached=True,
        has_simple_avoidance=True,
        obstacle_stop_behavior_ok=True,
        pre_obstacle_motion=False,
    )
    values.update(overrides)
    return values


class ObstacleCaseClassificationTest(unittest.TestCase):
    def test_pass_requires_goal_reached(self):
        self.assertEqual(classify_result(**evidence())[0], "PASS")
        self.assertEqual(classify_result(**evidence(goal_reached=False))[0], "STOPPED_AFTER_OBSTACLE")

    def test_safety_and_setup_categories_take_priority(self):
        self.assertEqual(classify_result(**evidence(mrm_operation_count=1))[0], "MRM")
        self.assertEqual(classify_result(**evidence(invalid_trajectory_count=1))[0], "INVALID_TRAJECTORY")
        self.assertEqual(classify_result(**evidence(min_collision_clearance=-0.01))[0], "COLLISION")
        self.assertEqual(classify_result(**evidence(object_count_ok=False))[0], "SETUP_INVALID")

    def test_rectangle_clearance_reports_overlap_for_identical_boxes(self):
        # Identical boxes overlap, so a conservative signed clearance is negative.
        self.assertLess(
            rectangle_signed_clearance(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0),
            0.0,
        )

    def test_rectangle_clearance_reports_separation_when_boxes_are_far_apart(self):
        # A separated-axis SAT metric must not use a negative lateral overlap
        # to hide the positive longitudinal gap.
        self.assertGreater(
            rectangle_signed_clearance(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 2.0, 1.0),
            7.0,
        )

    def test_rectangle_clearance_reports_touching_boxes_as_zero(self):
        separation = (2.23 + 2.0) / 2.0 + BASE_LINK_TO_VEHICLE_CENTER_M
        self.assertAlmostEqual(
            rectangle_signed_clearance(0.0, 0.0, 0.0, separation, 0.0, 0.0, 2.0, 1.0),
            0.0,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
