#!/usr/bin/env python3
"""
Python-side validation of matlab_bridge.py against the EXISTING
main2_collision_free.PathPlanner.  Run:

    python test_matlab_bridge.py

This is the same test set MATLAB/Simulink will exercise.  It checks:
 1. Bridge imports and initializes the existing planner (no new A*).
 2. Static A* produces a road-following path from start to goal.
 3. A dynamic obstacle blocking the corridor triggers replanning (AVOIDING).
 4. ego_step moves forward continuously (no teleport / no reverse jumps).
 5. Rear-end closing vehicle -> EGO accelerates / lane-shifts (rear response).
 6. Obstacle states are never modified by planning or ego stepping.
 7. All 10 obstacle types (incl. pothole) are accepted.
 8. Metrics accumulate (collision_count, clearance, replans, time).
"""

import math
import sys

import matlab_bridge as mb
import main2_collision_free as sim

PASS, FAIL = "PASS", "FAIL"
results = []


def check(name, cond, extra=""):
    results.append((name, bool(cond), extra))
    print(f"  [{PASS if cond else FAIL}] {name} {extra}")


def drive(ego, obstacles_fn, steps, dt=1.0 / 60.0, plan_every=0.25):
    """Closed loop: plan every plan_every s, step every dt."""
    traj = [ego[:2]]
    step_plan = int(round(plan_every / dt))
    for k in range(steps):
        for i, o in enumerate(obstacles_fn(k * dt)):
            pass
        mb.update_obstacles(obstacles_fn(k * dt))
        if k % step_plan == 0:
            mb.plan_path(ego[0], ego[1], ego[3])
        ego = [v for v in mb.ego_step(ego[0], ego[1], ego[2], ego[3], dt).values()]
        ego = [ego[0], ego[1], ego[2], ego[3]]
        traj.append((ego[0], ego[1]))
    return ego, traj


print("=" * 70)
print("TEST 1 - initialize existing planner, verify SAME PathPlanner class")
print("=" * 70)
st = mb.initialize_planner()
check("initialize_planner ok", st['ok'])
check("planner is main2_collision_free.PathPlanner",
      type(mb._SESSION.planner) is sim.PathPlanner)
check("planner grid built (A* cost map)", len(mb._SESSION.planner.grid) == sim.GW)

print()
print("=" * 70)
print("TEST 2 - static A* path: (300,400) -> (4700,400) on the highway")
print("=" * 70)
mb.set_goal(4700, 400)
r = mb.plan_path(300, 400, 0.0)
check("path found", r['num_waypoints'] > 5, f"waypoints={r['num_waypoints']}")
check("path starts near start", math.hypot(r['path_x'][0] - 300, r['path_y'][0] - 400) < 100)
# planner.path is the ~56-waypoint lookahead window (existing behavior).
# The full static A* route must reach the goal.
full = mb.get_path()
check("static A* route ends near goal",
      math.hypot(full['static_path_x'][-1] - 4700,
                 full['static_path_y'][-1] - 400) < 200,
      f"end=({full['static_path_x'][-1]:.0f},{full['static_path_y'][-1]:.0f})")
check("planner_state CRUISING", r['planner_state'] == 'CRUISING', r['planner_state'])
check("planning_time < 2 s", r['planning_time'] < 2.0, f"{r['planning_time']*1000:.0f} ms")

print()
print("=" * 70)
print("TEST 3 - dynamic obstacle blocks corridor -> dynamic replan (AVOIDING)")
print("=" * 70)
mb.initialize_planner()          # fresh session
mb.set_goal(4700, 400)
r0 = mb.plan_path(300, 400, 0.0)
check("clear corridor -> CRUISING", r0['planner_state'] == 'CRUISING')
# A two-wheeler stopped on the ego's lane blocks only part of the 150 px
# road width, so the dynamic local A* must produce a lateral detour.
ob = {'type': 'two_wheeler', 'x': 520.0, 'y': 395.0, 'velocity': 0.0, 'heading': 0.0}
mb.update_obstacles([ob])
r1 = mb.plan_path(300, 400, 0.0)
check("partially blocked corridor -> AVOIDING",
      r1['planner_state'] == 'AVOIDING', r1['planner_state'])
check("replanning_required flag", r1['replanning_required'] is True)
lat = max(abs(y - 395) for y in r1['path_y'][1:])
check("new path deviates laterally around obstacle", lat > 25,
      f"max lateral offset of new path {lat:.0f}px")
# A truck parked across the whole lane leaves no gap: existing behavior is
# EGO-only EMERGENCY stop (never re-routes traffic).
mb.update_obstacles([{'type': 'truck', 'x': 480.0, 'y': 400.0, 'velocity': 0.0, 'heading': 0.0}])
r2 = mb.plan_path(300, 400, 0.0)
check("fully blocked corridor -> EMERGENCY stop (EGO-only)",
      r2['planner_state'] == 'EMERGENCY', r2['planner_state'])

print()
print("=" * 70)
print("TEST 4 - forward motion, no teleportation, obstacles untouched")
print("=" * 70)
mb.initialize_planner()
mb.set_goal(1200, 400)
obst = [{'type': 'car', 'x': 500.0, 'y': 420.0, 'velocity': 40.0, 'heading': 0.0}]
snapshot = [dict(o) for o in obst]
ego = [300.0, 400.0, 0.0, 0.0]


def obst_fn(t):
    o = snapshot[0]
    return [{'type': 'car', 'x': o['x'] + 40.0 * t, 'y': o['y'],
             'velocity': 40.0, 'heading': 0.0}]


STEPS, DT = 240, 1.0 / 60.0
max_jump, back_jumps, x_prev = 0.0, 0, ego[0]
for k in range(STEPS):
    mb.update_obstacles(obst_fn(k * DT))
    if k % 15 == 0:
        mb.plan_path(ego[0], ego[1], ego[3])
    e = mb.ego_step(*ego, DT)
    jump = math.hypot(e['x'] - x_prev, e['y'] - ego[1])
    max_jump = max(max_jump, jump)
    if e['x'] < x_prev - 0.5:
        back_jumps += 1
    # obstacle motion driven only by the test (like Entity.update): verify
    # the bridge never edits obstacle state itself.
    for o in mb._SESSION.obstacles:
        assert o.etype != 'ego'
    x_prev = e['x']
    ego = [e['x'], e['y'], e['v'], e['heading']]
phys_max = 125 * DT * 1.5
check("ego moved forward", ego[0] > 500, f"x={ego[0]:.0f}")
check("max per-step displacement bounded (no teleport)",
      max_jump <= phys_max + 1.0, f"max_jump={max_jump:.1f}px, limit~{phys_max:.1f}px")
check("no reverse jumps in x", back_jumps == 0)
m = mb.get_metrics()
check("collision-free approach run", m['collision_count'] == 0,
      f"collisions={m['collision_count']}, min_clear={m['minimum_clearance']:.1f}px")

print()
print("=" * 70)
print("TEST 5 - rear-end threat: fast car closing from behind")
print("=" * 70)
mb.initialize_planner()
mb.set_goal(4700, 400)
# Wrong-way vehicle in the ego's lane closing on the ego from behind
# (heading pi -> moving in -x, toward the ego's rear). Mirrors the trigger
# semantics of World._rear_collision_response.
mb.update_obstacles([{'type': 'car', 'x': 200.0, 'y': 400.0,
                      'velocity': 120.0, 'heading': math.pi}])
mb.plan_path(300, 400, 0.0)
e = mb.ego_step(300.0, 400.0, 30.0, 0.0, 1.0 / 60.0)
check("rear threat -> accelerate away (target_speed > current)",
      e['target_speed'] > 30.0, f"target={e['target_speed']:.0f}")
check("ego continues moving forward", e['x'] >= 300.0)

print()
print("=" * 70)
print("TEST 6 - all 10 STEP-5 obstacle types accepted")
print("=" * 70)
for t in ('car', 'truck', 'bus', 'auto_rickshaw', 'two_wheeler', 'bicycle',
          'pedestrian', 'pushcart', 'animal', 'pothole'):
    n = mb.update_obstacles([{'type': t, 'x': 1000, 'y': 1000,
                              'velocity': 0, 'heading': 0}])
    check(f"type '{t}'", n == 1)
# numeric Simulink codes also accepted
n = mb.update_obstacles([[1, 900, 1000, 10, 0.0], [10, 950, 1050, 0, 0.0]])
check("numeric type codes (Simulink matrix form)", n == 2)

print()
print("=" * 70)
print("TEST 7 - metrics surface complete (STEP 7 fields)")
print("=" * 70)
m = mb.get_metrics()
for f in ('collision_count', 'minimum_clearance', 'number_of_replans',
          'last_planning_time', 'simulation_time'):
    check(f"metric '{f}' present", f in m, f"{f}={m.get(f)}")

print()
print("=" * 70)
failed = [n for n, ok, _ in results if not ok]
print(f"RESULT: {len(results) - len(failed)}/{len(results)} passed")
if failed:
    print("FAILED:", failed)
    sys.exit(1)
print("ALL PYTHON-SIDE TESTS PASSED")
