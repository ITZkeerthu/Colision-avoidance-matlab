# MATLAB / Simulink Integration — Adaptive Path Planning & Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

## Status: fully tested end-to-end on this machine (MATLAB R2026a + Python 3.12)

| Layer | Test | Result |
|---|---|---|
| Python bridge (`test_matlab_bridge.py`) | 35 assertions | **35/35 PASS** |
| MATLAB → Python A* (`run_planner_demo.m`) | 5 scenarios | **0 collisions total** |
| Simulink → Python A* (`adaptive_path_planning.slx` via `run_simulink_demo.m`) | closed-loop, 20 s | **0 collisions, PASS** |
| Pygame app (`main2_collision_free.py`) | launch + run | **runs unchanged** |

> The existing Pygame simulation is **unmodified** and remains the primary
> working demonstration. MATLAB/Simulink is an integration and validation
> layer *around* the existing A* planner — not a replacement.

---

## 1. Architecture

```text
                     MATLAB / SIMULINK
        ┌──────────────────────────────────────────┐
        │  Scenario / Environment State            │  (MATLAB Function block:
        │        │                                 │   obstacle matrix + goal)
        │        ▼                                 │
        │  Obstacle State ──► Prediction           │  (constant-velocity
        │        │                  │              │   prediction + TTC)
        │        ▼                  ▼              │
        │  Python A* Planner                       │  (Python Code block →
        │        │                                 │   EXISTING PathPlanner)
        │        ▼                                 │
        │  EGO Motion                              │  (Python Code block →
        │        │                                 │   bridge ego_step:
        │        ▼                                 │   bounded accel/steering)
        │  Collision / Safety Validation           │  (MATLAB Function block:
        │        │                                 │   clearance + counters)
        │        ▼                                 │
        │  Metrics / Scopes / To-Workspace         │
        └──────────────────┬───────────────────────┘
                           │  py.* / Python Code block (InProcess)
                           ▼
        ┌──────────────────────────────────────────┐
        │  matlab_bridge.py                        │
        │  thin API: initialize_planner / set_goal │
        │  update_obstacles / plan_path / get_path │
        │  ego_step / safety_check / get_metrics   │
        └──────────────────┬───────────────────────┘
                           │  imports (NO re-implementation)
                           ▼
        ┌──────────────────────────────────────────┐
        │  main2_collision_free.py                 │
        │  • existing PathPlanner (static A* +     │
        │    dynamic local A* + prediction tubes)  │
        │  • continuous-time collision gate        │
        │    (World._collision_free_fraction)      │
        │  • road network, 10 obstacle types,      │
        │    potholes, 10 Indian-road scenarios    │
        └──────────────────┬───────────────────────┘
                           │
                           ▼
                      Pygame window
                (primary judge demo — untouched)
```

Key design rule: **A\* never writes `ego.x / ego.y` directly.** It produces a
waypoint sequence; the EGO motion controller follows those waypoints with
bounded acceleration (180 px/s²), braking (520–700 px/s²) and steering rate
(5.5 rad/s), exactly like the Pygame `World.update()` loop. There is no
teleportation and no re-routing of other vehicles, ever.

---

## 2. How the Python A* connects to MATLAB

`main2_collision_free.py` ends with `if __name__ == "__main__": main()`, so
MATLAB can import it as a module **without opening the Pygame window**.

`matlab_bridge.py` exposes the existing planner:

| Function | Purpose |
|---|---|
| `initialize_planner(road_filter, pothole_density)` | builds the **existing** `PathPlanner` with the existing road network |
| `set_goal(gx, gy)` | delegates to `PathPlanner.set_goal` |
| `update_obstacles(obstacles)` | injects ground-truth obstacle states; accepts dicts, `[type, x, y, velocity, heading]` rows, or numeric Simulink type codes (0 = empty slot) |
| `plan_path(x, y, heading)` | calls the **existing** `PathPlanner.plan()` → `path_x, path_y, num_waypoints, path_length_px/_m, planner_state(_code), replanning_required, replanned_this_call, planning_time` |
| `get_path()` | current local path **and** the full static A* route (for plotting) |
| `get_planner_status()` | state, goal, waypoint counts, plan/replan counters |
| `ego_step(x, y, v, heading, dt, max_speed)` | one motion step: `control()` + rear-threat response + bounded kinematics + continuous collision gate (reuses `World._collision_free_fraction`) → new `[x, y, v, heading]`, `collision`, `min_clearance` |
| `safety_check(x, y)` | nearest-obstacle clearance |
| `get_metrics() / reset_metrics()` | `collision_count`, `minimum_clearance(_m)`, `number_of_replans`, `plan_calls`, `last_planning_time`, `simulation_time` |

Obstacle types supported (STEP 5): `car, truck, bus, auto_rickshaw,
two_wheeler, bicycle, pedestrian, pushcart, animal, pothole` (codes 1–10).
Units: simulation pixels, `PX_PER_M = 10` (10 px = 1 m).

MATLAB side usage (this is exactly what `run_planner_demo.m` does):

```matlab
if count(py.sys.path, pwd) == 0, insert(py.sys.path, int32(0), pwd); end
mb = py.importlib.import_module('matlab_bridge');
mb.initialize_planner();
mb.set_goal(4700, 400);
mb.update_obstacles(py.list({py.dict(pyargs('type','two_wheeler', ...
    'x',700,'y',340,'velocity',0,'heading',0))}));
r  = mb.plan_path(300, 400, 0.0);          % EXISTING A* does the work
px = cellfun(@double, cell(r{'path_x'})).';
py = cellfun(@double, cell(r{'path_y'})).';
plot(px/10, py/10);
```

---

## 3. How the Simulink model works

`adaptive_path_planning.slx` (built by `build_simulink_model.m`, MATLAB
R2025a+ required for the **Python Code** block) — or skip the reading and
just double-click **`run_everything.bat`**: it launches the Pygame window,
runs both MATLAB/Simulink validations with figures, then opens the Simulink
model tab and starts a live 20 s run with updating Display blocks.

| Block | Type | Role |
|---|---|---|
| `Simulation Time` | Clock | drives the scenario |
| `Scenario Environment State` | MATLAB Function | obstacle matrix (8×5: type,x,y,v,heading) + goal; analytic Indian-road traffic (EGO trajectories never touched) |
| `Obstacle State` | Mux | state distribution |
| `Prediction` | MATLAB Function | 1.5 s constant-velocity prediction + time-to-collision |
| `Python AStar Planner` | **Python Code** | `InitializeConditionsCode` imports `matlab_bridge` and creates the **existing** `PathPlanner`; `OutputCode` (every 0.2 s = periodic replanning) calls `plan_path()` → `path_x[60], path_y[60], num_wp, path_len, state_code, replan_flag, plan_time`. Persistent `PythonObject` symbol holds the bridge module |
| `EGO Motion` | **Python Code** | every 0.02 s calls `mb.ego_step(...)` — the same bounded-acceleration / steering-rate motion logic as the Pygame world; **no teleportation** |
| `EGO State Delay` | Unit Delay | closes the physical feedback loop (IC `[300;400;0;0]`) |
| `Collision Safety Validation` | MATLAB Function | independent clearance / collision counting (same radii as the bridge) |
| `Replan Counter` | MATLAB Function | counts replanning events |
| Displays / Scope / To Workspace | sinks | `ego_log`, `planner_log`, `safety_log`, collisions, clearance, planner state |

Solver: fixed-step `FixedStepDiscrete`, `FixedStep = 0.02`, stop time 20 s.
Planner sample time 0.2 s (periodic + event-based replanning: a replan is
forced whenever the planner reports `AVOIDING`/`EMERGENCY`).

Simulink signals arrive in the Python snippets as `numpy.ndarray`; snippets
return numpy arrays / floats to the output ports (data typing verified on
this machine).

---

## 4. How to configure pyenv

On this machine pyenv is already configured:

```text
Version: 3.12
Executable: C:\Users\itzke\AppData\Local\Programs\Python\Python312\pythonw.exe
ExecutionMode: InProcess
```

Check with:

```matlab
pyenv
```

If it is `Unloaded`/wrong, run once per MATLAB installation:

```matlab
pyenv('Version', ...
  'C:\Users\itzke\AppData\Local\Programs\Python\Python312\python.exe')
```

The Python interpreter needs `pygame`, `numpy`, `pygltflib`
(`pip install pygame numpy pygltflib`). MATLAB must use the same
interpreter (`InProcess` mode is the default).

### Environment note (only if a Simulink build errors with "…_cgxe.bat is not recognized")

Some sandboxed shells set the environment variable
`NoDefaultCurrentDirectoryInExePath=1`, which prevents cmd from running a
batch file by bare name from the current directory — exactly how Simulink
invokes its generated build script. Fix for that shell before starting
MATLAB:

```powershell
[Environment]::SetEnvironmentVariable('NoDefaultCurrentDirectoryInExePath', $null)
& "C:\Program Files\MATLAB\R2026a\bin\matlab.exe"
```

Interactive MATLAB sessions started from the Desktop/Start menu are normally
not affected. Build scripts in this project also redirect Simulink's cache
to a local folder (`Simulink.fileGenControl`) because mapped/network drives
can break the simulation-target build. See `ENCOUNTERED_ERRORS.txt`.

---

## 5. How to run the MATLAB demo

From MATLAB (or batch):

```matlab
cd <project folder>
run_planner_demo
```

or headless:

```powershell
matlab -batch "run_planner_demo" -sd <project folder>
```

What it does: configures `pyenv`, imports `matlab_bridge`, runs **five**
closed-loop scenarios (below) through the existing A\*, converts the Python
results to MATLAB arrays, prints a metrics table, and saves
`planner_demo_scenarios.png` + `planner_demo_replan.png` (validation plots
only — Pygame remains the live visualization).

---

## 6. How to open and run the Simulink model

The model file `adaptive_path_planning.slx` is already generated. To open
and run it interactively:

```matlab
cd <project folder>
open_system('adaptive_path_planning.slx')
sim('adaptive_path_planning')      % or press Run
```

Watch the **Display** blocks (`Collisions`, `Min Clearance (px)`,
`Planner State`, `Replan Events`) and the **Metrics Scope**.

To regenerate the model from scratch (e.g. after moving the project):

```matlab
build_simulink_model      % re-creates adaptive_path_planning.slx
```

To run the validated end-to-end sweep with plots and assertions:

```matlab
run_simulink_demo         % simulates + independent checks + PNG
```

---

## 7. How the five scenarios are validated

`run_planner_demo.m` drives these five Indian-road scenarios in a closed
loop (ground-truth obstacles move analytically — never modified — while the
existing planner replans and the EGO follows with bounded motion):

| # | Scenario | Challenge | Demonstrates | Measured result (this machine) |
|---|---|---|---|---|
| 1 | Highway Cruise | slower car + truck ahead | collision-free following/overtake path | 0 collisions, 6.15 m min clearance |
| 2 | Two-Wheeler Blockage | stopped two-wheeler on lane | dynamic local A* lateral detour | 0 collisions, 3.48 m |
| 3 | Pedestrian Crossing | pedestrian walks across lane | yield (full stop) then resume | 0 collisions, 0.92 m |
| 4 | Rear-End Evasion | wrong-way car closing from behind | EGO accelerate/lane-shift response | 0 collisions, 13.78 m |
| 5 | Dense Mixed Traffic | auto-rickshaw, car, bicycle, pothole, two-wheeler | adaptive replanning in clutter | 0 collisions, 3.89 m |

The Simulink model itself runs one integrated scenario containing three of
these challenges at once: stopped two-wheeler (**static**, Test-2 style),
a moving car in the lane (**moving vehicle**, Test-3 style) and a crossing
pedestrian (**Test-4** style) — validated end-to-end in Simulink with:
`0 collisions, 3.80 m minimum clearance, 8 replanning events,
max per-step displacement 2.50 px (= physical speed limit → no teleport),
no reverse jumps`. See `simulink_validation.png`.

The rear-end scenario is additionally covered in the bridge test suite
(`test_matlab_bridge.py`, Test 5) and in scenario 4 of the MATLAB demo.

For the full live experience (10 Indian-road scenarios, GLB models, night &
monsoon conditions), run the primary demo:

```powershell
python main2_collision_free.py      # Pygame simulation, keys 1-9,0
```

---

## 8. How collision-free performance is measured

Metrics are produced at three levels, all consistent:

**Python bridge** (`mb.get_metrics()`, used by both MATLAB and Simulink):
`collision_count`, `minimum_clearance(_m)`, `number_of_replans`,
`plan_calls`, `last_planning_time`, `simulation_time`.

**Simulink model** (logged via To Workspace): `ego_log` ([x y v heading]),
`planner_log` ([num_wp, path_length_m, state_code, replan_flag,
planning_time_ms, min_clearance_now]), `safety_log` (predicted obstacle
states); plus Display counters and a metrics Scope.

**Independent MATLAB validation** (`run_simulink_demo.m`): recomputes
clearances analytically from the known obstacle trajectories (not trusting
the planner's own accounting) and asserts:

| Metric | Rule |
|---|---|
| `collision_count` | center-distance − (ego radius + obstacle radius) < 0 → hit; **must be 0** |
| `minimum_clearance` | min over all steps/obstacles of that signed gap (reported in px and m) |
| `number_of_replans` | rising edges of `replanned_this_call` / state entering `AVOIDING` |
| `path_length` | sum of segment lengths of the current A* path (`PX_PER_M = 10 px/m`) |
| `planning_time` | wall time of the Python `plan()` call (ms; mean ≈ 3 ms measured) |
| `simulation_time` | Simulink clock / accumulated `dt` |
| no teleport | every per-step displacement ≤ `max_speed * dt` (125 × 0.02 = 2.5 px) |
| no reverse jumps | `x` of EGO never decreases by more than 0.5 px (heading 0 scenarios) |

---

## File inventory

| File | Status |
|---|---|
| `main2_collision_free.py` | **existing solution, unmodified** (Pygame + A* + all scenarios) |
| `matlab_bridge.py` | new — thin API over the existing `PathPlanner` |
| `run_planner_demo.m` | new — MATLAB demo, 5 scenarios, plots, metrics table |
| `build_simulink_model.m` | new — programmatic generator for the model |
| `adaptive_path_planning.slx` | new — the Simulink model (built & simulated) |
| `run_simulink_demo.m` | new — Simulink validation + assertions + PNG |
| `test_matlab_bridge.py` | new — 35 Python-side assertions (run: `python test_matlab_bridge.py`) |
| `diag_simulink.m` | helper — dumps ego/planner timelines from the model |
| `run_everything.bat` / `run_everything.m` | **one-click full demo** — Pygame window + all MATLAB/Simulink validations + live Simulink run |
| `ENCOUNTERED_ERRORS.txt` | record of the environment issues hit & fixes |

Nothing in `main2_collision_free.py`, the obstacle behavior, the A*
implementation, or the Pygame visualization was changed.
