#!/usr/bin/env python3
"""
Unit tests for the obstacle-input handling of matlab_bridge (_norm_obstacle,
update_obstacles, safety_check).  These guard the Simulink/MATLAB boundary:
empty or malformed obstacle-matrix rows must be ignored, never crash planning.

    python test_obstacle_normalization.py
"""

import math
import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import matlab_bridge as mb  # noqa: E402


def test_dict_and_list_forms():
    d = mb._norm_obstacle({"type": "Truck", "x": 10, "y": 20, "velocity": 3, "heading": 1.5})
    assert d.etype == "truck" and (d.x, d.y, d.speed, d.angle) == (10.0, 20.0, 3.0, 1.5)
    row = mb._norm_obstacle(["car", 1, 2, 3, 4])
    assert row.etype == "car" and row.max_speed == 3.0


def test_unknown_type_falls_back_to_car():
    assert mb._norm_obstacle(["spaceship", 0, 0, 0, 0]).etype == "car"


def test_numeric_codes():
    for name, code in mb.obstacle_type_codes().items():
        assert mb._norm_obstacle([code, 0, 0, 0, 0]).etype == name
    assert mb._norm_obstacle([0, 0, 0, 0, 0]) is None      # empty Simulink slot
    assert mb._norm_obstacle([999, 0, 0, 0, 0]) is None    # unknown code


def test_malformed_rows_ignored():
    assert mb._norm_obstacle(None) is None
    assert mb._norm_obstacle("car") is None
    assert mb._norm_obstacle(["car", 1, 2]) is None                 # too short
    assert mb._norm_obstacle(["car", "a", 2, 3, 4]) is None         # non-numeric
    assert mb._norm_obstacle(["car", float("nan"), 2, 3, 4]) is None
    assert mb._norm_obstacle(["car", 1, float("inf"), 3, 4]) is None
    assert mb._norm_obstacle([float("nan"), 1, 2, 3, 4]) is None
    assert mb._norm_obstacle(["ego", 1, 2, 3, 4]) is None


def test_update_obstacles_counts_only_valid_rows():
    mb.initialize_planner()
    n = mb.update_obstacles([["car", 100, 100, 1, 0], [0, 0, 0, 0, 0],
                             ["truck", float("nan"), 0, 0, 0], {"type": "bus", "x": 5, "y": 5}])
    assert n == 2, n


def test_safety_check_clearance_and_collision():
    mb.initialize_planner()
    mb.update_obstacles([])
    far = mb.safety_check(0, 0)
    assert math.isinf(far["min_clearance"]) and not far["collision"] and far["nearest_obstacle"] == ""
    mb.update_obstacles([["car", 50, 50, 0, 0]])
    hit = mb.safety_check(50, 50)
    assert hit["collision"] and hit["min_clearance"] < 0 and hit["nearest_obstacle"] == "car"
    assert not mb.safety_check(500, 500)["collision"]


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"[PASS] {name}")
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print(f"[FAIL] {name}: {type(exc).__name__}: {exc}")
    sys.exit(1 if failed else 0)
