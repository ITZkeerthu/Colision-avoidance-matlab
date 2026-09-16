#!/usr/bin/env python3
"""
matlab_bridge.py
================
Minimal Python API that exposes the EXISTING A* planner (PathPlanner) from
main2_collision_free.py to MATLAB / Simulink.

This module does NOT re-implement A*, obstacle behavior, or the EGO
controller.  It wraps the existing code:

    * Path planning      -> main2_collision_free.PathPlanner (static A* +
                            dynamic local A*, unchanged)
    * Continuous safety  -> main2_collision_free.World._collision_free_fraction
                            (the same continuous-time hard gate used by the
                            Pygame simulation, reused as a static method)
    * EGO motion         -> the same bounded acceleration / steering-rate
                            integration used by World.update(), reproduced
                            here so MATLAB/Simulink can drive a single EGO
                            agent headlessly.  The Pygame simulation itself
                            is untouched.

MATLAB/Simulink calls the small function surface below.  All distances are
in the simulation's pixel units unless suffixed _m (PX_PER_M = 10 px/m).

Public API (module level, one persistent session):
    initialize_planner(road_filter="", pothole_density=0.0) -> dict
    set_goal(gx, gy)                                        -> bool
    update_obstacles(obstacles)                             -> int
    plan_path(ego_x, ego_y, ego_heading=0.0)                -> dict
    get_path()                                              -> dict
    get_planner_status()                                    -> dict
    ego_step(x, y, v, heading, dt=..., max_speed=...)       -> dict
    safety_check(x, y)                                      -> dict
    get_metrics()                                           -> dict
    reset_metrics()                                         -> None
    obstacle_type_codes()                                   -> dict
"""

import math
import time
from typing import Dict, List, Optional, Sequence

import main2_collision_free as sim

# ── UNITS / TYPE TABLES ─────────────────────────────────────────────────
PX_PER_M = 10.0                      # 10 simulation pixels = 1 metre

# Obstacle physical sizes, mirroring sim.ESIZES plus a 'pothole' entry so the
# full STEP-5 type list is supported without touching the original tables.
OBSTACLE_SIZES = dict(sim.ESIZES)
OBSTACLE_SIZES['pothole'] = (30, 30)
OBSTACLE_TYPES = [t for t in ('car', 'truck', 'bus', 'auto_rickshaw',
                              'two_wheeler', 'bicycle', 'pedestrian',
                              'pushcart', 'animal', 'pothole')]

# Numeric codes used on the Simulink side (numeric matrices are easier to
# pass through ports than strings).  1-based for MATLAB friendliness.
TYPE_TO_CODE = {t: i + 1 for i, t in enumerate(OBSTACLE_TYPES)}
CODE_TO_TYPE = {v: k for k, v in TYPE_TO_CODE.items()}

# Planner states -> stable integer codes for Simulink/To-Workspace logging.
STATE_CODES = {'IDLE': 0, 'CRUISING': 1, 'AVOIDING': 2,
               'EMERGENCY': 3, 'NO_PATH': 4}
EGO_RADIUS = 23.0                    # same value used by PathPlanner defaults


class _ObstacleStub:
    """
    Duck-typed stand-in for sim.Entity, exposing exactly the attributes the
    existing PathPlanner / safety gate read.  The real Entity class and its
    behavior are NOT modified; these stubs only carry state supplied by MATLAB
    (ground-truth environment representation).
    """
    __slots__ = ('etype', 'x', 'y', 'prev_x', 'prev_y', 'speed', 'angle',
                 'radius', 'max_speed', 'alive')

    def __init__(self, etype: str, x: float, y: float,
                 speed: float, angle: float):
        w, h = OBSTACLE_SIZES.get(etype, (20, 20))
        self.etype = etype
        self.x = float(x)
        self.y = float(y)
        self.prev_x = self.x
        self.prev_y = self.y
        self.speed = float(speed)
        self.angle = float(angle)
        self.radius = max(w, h) * .55
        self.max_speed = abs(float(speed))
        self.alive = True


def _norm_obstacle(obs) -> Optional[_ObstacleStub]:
    """Accept {dict} or [type, x, y, v, heading]; return a stub or None."""
    if isinstance(obs, dict):
        etype = obs.get('type') or obs.get('etype')
        x, y = obs.get('x', 0.0), obs.get('y', 0.0)
        v = obs.get('velocity', obs.get('speed', 0.0))
        hdg = obs.get('heading', obs.get('angle', 0.0))
    elif isinstance(obs, (list, tuple)) and len(obs) >= 5:
        etype, x, y, v, hdg = obs[0], obs[1], obs[2], obs[3], obs[4]
    else:
        return None
    # Numeric Simulink codes allowed in place of names.  Code 0 / unknown =
    # empty slot in the fixed-size Simulink obstacle matrix -> ignored.
    if isinstance(etype, (int, float)):
        etype = CODE_TO_TYPE.get(int(etype))
        if etype is None:
            return None
    etype = str(etype).strip().lower().replace(' ', '_').replace('-', '_')
    if etype == 'ego':
        return None
    if etype not in OBSTACLE_SIZES:
        etype = 'car'
    return _ObstacleStub(etype, x, y, v, hdg)


class _PlannerSession:
    """One persistent A* planning session shared by MATLAB and Simulink."""

    def __init__(self):
        self.planner: Optional['sim.PathPlanner'] = None
        self.obstacles: List[_ObstacleStub] = []
        self.goal = None
        self._now = 0.0                    # internal planner clock (seconds)
        # ── metrics ──
        self.plan_calls = 0
        self.replan_count = 0              # transitions INTO AVOIDING/EMERG/NO_PATH
        self._prev_state = 'IDLE'
        self.last_planning_time = 0.0
        self.collision_count = 0
        self.min_clearance = float('inf')
        self.sim_time = 0.0

    # ── setup ──
    def init(self, road_filter: str = "", pothole_density: float = 0.0):
        rf = road_filter.strip() or None
        potholes = sim.gen_potholes(sim.ROADS, pothole_density) \
            if pothole_density and pothole_density > 0 else []
        self.planner = sim.PathPlanner(sim.ROADS, potholes, rf)
        self.__init_metrics_only()
        return {'ok': True, 'road_filter': rf or '',
                'potholes': len(potholes)}

    def __init_metrics_only(self):
        self.obstacles = []
        self.goal = None         # never leak a goal across initialize_planner()
        self._now = 0.0
        self.plan_calls = 0
        self.replan_count = 0
        self._prev_state = 'IDLE'
        self.last_planning_time = 0.0
        self.collision_count = 0
        self.min_clearance = float('inf')
        self.sim_time = 0.0

    def require(self) -> 'sim.PathPlanner':
        if self.planner is None:
            self.init()
        return self.planner

    # ── world-state input ──
    def update_obstacles(self, obstacles: Sequence) -> int:
        stubs = []
        old = {(o.etype, round(o.x / 40), round(o.y / 40)): o
               for o in self.obstacles}
        for raw in obstacles:
            st = _norm_obstacle(raw)
            if st is None:
                continue
            # Preserve previous position of the matching obstacle so the
            # continuous-time safety gate sees a real prev->current segment,
            # exactly like the Pygame world's entities.
            key = (st.etype, round(st.x / 40), round(st.y / 40))
            prev = old.get(key)
            if prev is not None:
                st.prev_x, st.prev_y = prev.x, prev.y
            stubs.append(st)
        self.obstacles = stubs
        return len(stubs)

    # ── planning (EXISTS in main2_collision_free.PathPlanner) ──
    def plan_path(self, ex: float, ey: float, heading: float = 0.0):
        p = self.require()
        if self.goal is None:
            raise RuntimeError('set_goal() must be called before plan_path()')
        # Self-heal: if the underlying planner was rebuilt after the goal was
        # registered (e.g. initialize_planner() called between plan calls),
        # re-apply the session goal so a fresh planner never plans to None.
        if p.goal is None or tuple(p.goal) != tuple(self.goal):
            p.set_goal(float(self.goal[0]), float(self.goal[1]))

        prev_path = p.path
        t0 = time.perf_counter()
        # force=True -> the caller (MATLAB/Simulink) decides WHEN to replan;
        # the existing 0.25 s self-throttle is kept available via plan_interval.
        p.plan(float(ex), float(ey), self.obstacles,
               ego_radius=EGO_RADIUS, now=self._now, force=True)
        self.last_planning_time = time.perf_counter() - t0
        self.plan_calls += 1

        replanned = bool(p.path) and p.path != prev_path \
            and p.state in ('AVOIDING', 'EMERGENCY', 'NO_PATH')
        if p.state in ('AVOIDING', 'EMERGENCY', 'NO_PATH') \
                and self._prev_state not in ('AVOIDING', 'EMERGENCY', 'NO_PATH'):
            self.replan_count += 1
        self._prev_state = p.state

        xs = [float(pt[0]) for pt in p.path]
        ys = [float(pt[1]) for pt in p.path]
        length_px = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                        for i in range(1, len(xs)))
        return {
            'path_x': xs,
            'path_y': ys,
            'num_waypoints': len(xs),
            'path_length_px': float(length_px),
            'path_length_m': float(length_px / PX_PER_M),
            'planner_state': p.state,
            'planner_state_code': STATE_CODES.get(p.state, -1),
            'replanning_required': bool(p.state in ('AVOIDING', 'EMERGENCY',
                                                    'NO_PATH')),
            'replanned_this_call': bool(replanned),
            'planning_time': float(self.last_planning_time),
        }

    # ── EGO motion / safety (mirrors World.update EGO logic, Simulink-free) ──
    def ego_step(self, x, y, v, heading, dt=1.0 / 60.0, max_speed=None):
        p = self.require()
        dt = float(dt)
        dt = min(max(dt, 1e-4), 0.12)          # same clamp as World.update
        max_speed = sim.ESPEEDS['ego'][1] if max_speed is None else float(max_speed)

        ux, uy, target_speed, threat = p.control(
            float(x), float(y), EGO_RADIUS, self.obstacles, max_speed,
            float(heading))

        # Rear-closing response: mirror of World._rear_collision_response,
        # steering only the EGO toward a reachable point ahead and to one
        # side.  Stub obstacles keep their own motion – they are never edited.
        resp = self._rear_response(float(x), float(y), float(v), float(heading))
        if resp is not None:
            ux, uy, target_speed = resp

        v = float(v)
        if target_speed < v:
            v = max(target_speed, v - 520 * dt)
        else:
            v = min(target_speed, v + 180 * dt)
        if target_speed <= 0.1:
            v = max(0.0, v - 700 * dt)

        heading = float(heading)
        if v > 0.01 and (ux or uy):
            desired = math.atan2(uy, ux)
            delta = sim.ang_lerp(heading, desired, 1.0)
            max_turn = 5.5 * dt
            heading = sim.ang_lerp(heading, desired,
                                   min(1.0, max_turn / max(0.001, abs(delta))))

        start = (float(x), float(y))
        desired_pt = (
            sim.clamp(start[0] + math.cos(heading) * v * dt, 30, sim.WW - 30),
            sim.clamp(start[1] + math.sin(heading) * v * dt, 30, sim.WH - 30))

        # Hard continuous-time collision gate — the identical static method
        # used by the Pygame world.  Only the EGO endpoint is shortened; the
        # EGO is never teleported onto a path.
        safe_end = list(desired_pt)
        safe = True
        for e in self.obstacles:
            if not e.alive:
                continue
            margin = EGO_RADIUS + e.radius + 8.0
            f = sim.World._collision_free_fraction(start, safe_end, e, margin)
            if f < 0.999:
                safe = False
                safe_end = [start[0] + (safe_end[0] - start[0]) * max(0.0, f),
                            start[1] + (safe_end[1] - start[1]) * max(0.0, f)]

        actual = sim.dist(start, safe_end)
        nx, ny = float(safe_end[0]), float(safe_end[1])
        nv = actual / dt
        if not safe and actual < 1.0:
            nx, ny, nv = start[0], start[1], 0.0

        # Metrics: clearance to every obstacle + collision bookkeeping.
        min_clear = float('inf')
        for e in self.obstacles:
            if not e.alive:
                continue
            c = math.hypot(nx - e.x, ny - e.y) - (EGO_RADIUS + e.radius)
            min_clear = min(min_clear, c)
        if min_clear < self.min_clearance:
            self.min_clearance = min_clear
        collided = min_clear < 0.0
        if collided:
            self.collision_count += 1

        self.sim_time += dt
        return {
            'x': nx, 'y': ny, 'v': nv, 'heading': float(heading),
            'target_speed': float(target_speed), 'threat': float(threat),
            'collision': bool(collided),
            'min_clearance': float(min_clear),
            'min_clearance_m': float(min_clear / PX_PER_M),
            'safe_move': bool(safe),
            'planner_state': p.state,
            'planner_state_code': STATE_CODES.get(p.state, -1),
        }

    def _rear_response(self, x, y, v, heading):
        """Compact port of World._rear_collision_response (EGO-only steering)."""
        fx, fy = math.cos(heading), math.sin(heading)
        sx, sy = -fy, fx
        dt = 1.0 / 60.0
        threats = []
        for e in self.obstacles:
            if not e.alive:
                continue
            rx, ry = e.x - x, e.y - y
            longitudinal = rx * fx + ry * fy
            d = math.hypot(rx, ry)
            if longitudinal >= -25.0 or d > 300.0:
                continue
            ovx = (e.x - e.prev_x) / dt if (e.x, e.y) != (e.prev_x, e.prev_y) \
                else math.cos(e.angle) * e.speed
            ovy = (e.y - e.prev_y) / dt if (e.x, e.y) != (e.prev_x, e.prev_y) \
                else math.sin(e.angle) * e.speed
            closing = -((ovx - fx * v) * fx + (ovy - fy * v) * fy)
            ttc = (-longitudinal) / max(closing, 1e-3) if closing > 1.0 else 999.0
            if closing > 12.0 and (ttc < 2.6 or d < 150.0):
                threats.append((ttc, e))
        if not threats:
            return None
        threats.sort(key=lambda z: z[0])
        threat = threats[0][1]

        best = None
        for side in (-1, 1):
            tx = x + fx * 170.0 + sx * side * 48.0
            ty = y + fy * 170.0 + sy * side * 48.0
            if not sim.on_road(tx, ty, sim.ROADS, EGO_RADIUS + 4):
                continue
            clear = min((sim.dist((tx, ty), (e.x, e.y)) - EGO_RADIUS - e.radius
                         for e in self.obstacles if e.alive), default=1e9)
            threat_side = (threat.x - x) * sx + (threat.y - y) * sy
            score = clear + (30.0 if threat_side * side < 0 else 0.0)
            if best is None or score > best[0]:
                best = (score, tx, ty)
        if best is None:
            return None
        _, tx, ty = best
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < 1.0:
            return None
        desired_speed = min(sim.ESPEEDS['ego'][1],
                            max(v + 35.0, sim.ESPEEDS['ego'][1] * 0.92))
        return dx / d, dy / d, desired_speed


# ── MODULE-LEVEL FACADE (what MATLAB / Simulink actually calls) ──────────
_SESSION = _PlannerSession()


def initialize_planner(road_filter: str = "", pothole_density: float = 0.0):
    return _SESSION.init(road_filter, pothole_density)


def set_goal(gx: float, gy: float) -> bool:
    p = _SESSION.require()
    p.set_goal(float(gx), float(gy))
    _SESSION.goal = p.goal
    _SESSION._prev_state = 'IDLE'
    return True


def update_obstacles(obstacles) -> int:
    return _SESSION.update_obstacles(obstacles)


def plan_path(ego_x: float, ego_y: float, ego_heading: float = 0.0):
    _SESSION._now += 1.0                 # advance the internal planner clock
    return _SESSION.plan_path(ego_x, ego_y, ego_heading)


def get_path():
    """Current local path AND the full static A* route (for MATLAB plotting)."""
    p = _SESSION.require()
    return {'path_x': [float(pt[0]) for pt in p.path],
            'path_y': [float(pt[1]) for pt in p.path],
            'path_idx': int(p.path_idx),
            'num_waypoints': len(p.path),
            'static_path_x': [float(pt[0]) for pt in p.static_path],
            'static_path_y': [float(pt[1]) for pt in p.static_path]}


def get_planner_status():
    p = _SESSION.require()
    return {
        'planner_state': p.state,
        'planner_state_code': STATE_CODES.get(p.state, -1),
        'goal': list(p.goal) if p.goal else None,
        'num_waypoints': len(p.path),
        'static_waypoints': len(p.static_path),
        'plan_calls': _SESSION.plan_calls,
        'replan_count': _SESSION.replan_count,
        'last_planning_time': _SESSION.last_planning_time,
        'num_obstacles': len(_SESSION.obstacles),
    }


def ego_step(x, y, v, heading, dt=1.0 / 60.0, max_speed=None):
    return _SESSION.ego_step(x, y, v, heading, dt, max_speed)


def safety_check(x, y):
    min_clear = float('inf')
    nearest = ''
    for e in _SESSION.obstacles:
        if not e.alive:
            continue
        c = math.hypot(float(x) - e.x, float(y) - e.y) - (EGO_RADIUS + e.radius)
        if c < min_clear:
            min_clear, nearest = c, e.etype
    return {'min_clearance': float(min_clear),
            'min_clearance_m': float(min_clear / PX_PER_M),
            'collision': bool(min_clear < 0.0),
            'nearest_obstacle': nearest}


def get_metrics():
    mc = _SESSION.min_clearance
    return {
        'collision_count': int(_SESSION.collision_count),
        'minimum_clearance': float(mc if math.isfinite(mc) else -1.0),
        'minimum_clearance_m': float(mc / PX_PER_M if math.isfinite(mc) else -1.0),
        'number_of_replans': int(_SESSION.replan_count),
        'plan_calls': int(_SESSION.plan_calls),
        'last_planning_time': float(_SESSION.last_planning_time),
        'simulation_time': float(_SESSION.sim_time),
    }


def reset_metrics():
    s = _SESSION
    s.collision_count = 0
    s.min_clearance = float('inf')
    s.replan_count = 0
    s.plan_calls = 0
    s.last_planning_time = 0.0
    s.sim_time = 0.0
    s._prev_state = _SESSION.require().state


def obstacle_type_codes():
    """-> {'car': 1, 'truck': 2, ..., 'pothole': 10} for Simulink matrices."""
    return dict(TYPE_TO_CODE)
