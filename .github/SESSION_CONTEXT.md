# PID Controller — Session Context

## Repository
`/home/zenardi/Documents/develop/sdc-c6-control-starter`

```bash
# Build
cd project/pid_controller && cmake . && make

# Run simulation (CARLA must be running first)
cd project && ./run_main_pid.sh
```

CARLA is local at `project/CARLA/` (v0.9.16), map: **Town06_Opt** (highway).

---

## PID Implementation

**Files:** `project/pid_controller/pid_controller.h` / `pid_controller.cpp`

```cpp
// UpdateError(cte)
_p_error = cte;
_d_error = (cte - previous_p_error) / dt;   // dt capped at 0.5 s to prevent D saturation
_i_error += cte * dt;

// TotalError() — opposes error, clamped to actuator limits
control = -Kp*p_error - Ki*i_error - Kd*d_error;
```

**Tuned gains** (`project/pid_controller/main.cpp`):

| PID      | Kp   | Ki    | Kd   | Limits        |
|----------|------|-------|------|---------------|
| Steer    | 0.05 | 0.0   | 0.05 | [−0.3, +0.3]  |
| Throttle | 0.10 | 0.001 | 0.05 | [−1.0, +1.0]  |

**Steer error** (cross-track error, `main.cpp` ~line 341):
```cpp
error_steer = -sin(yaw) * dx + cos(yaw) * dy;
// dx/dy = ego position minus first trajectory waypoint
```

**Throttle error** = `velocity − v_points[0]`  
Positive → too slow → accelerate; negative → braking.

---

## Architecture

```
simulatorAPI.py (Python)  ←WebSocket→  main.cpp (C++)
  get_waypoint()                         BehaviorPlannerFSM
  SpawnNPC()                             MotionPlanner → 7 cubic spirals
  sends: x, y, yaw, vel,                CostFunctions: collision + closeness
         obst_x/y/yaw                   Best spiral → x_points / y_points
                                        PID tracks x_points[0] / v_points[0]
```

**Collision detection** (`planning_params.h`, `cost_functions.cpp`):
- 3 bounding circles per vehicle at longitudinal offsets `{−1.0, 1.0, 3.0}` m
- Each circle radius: **1.5 m** → collision triggered when distance < **3.0 m**
- `P_NUM_PATHS = 7`, `P_GOAL_OFFSET = 1.0 m` → spirals span **±3 m** lateral from goal
- `P_SPEED_LIMIT = 3.0 m/s`

---

## Spawn Geometry (Town06_Opt, spawn[4])

| Parameter | Value |
|-----------|-------|
| Ego spawn | `(600.9, −10.0)`, yaw ≈ −180° (drives WEST) |
| `get_right_vector()` at yaw=−180° | `(0, −1, 0)` |
| Ego lane centre | y ≈ **−10.0** |
| Adjacent lane centre (`get_right_lane()`) | y ≈ **−13.5** |

`SpawnNPC` offset convention (`simulatorAPI.py`):
```
new_y = waypoint_lane_centre_y + offset_y * (−1)
  offset_y = 0.0   → ego lane centre   (y = −10.0)
  offset_y = +3.5  → adjacent lane     (y = −13.5)
```

`SpawnNPC` now calls `world.map.get_waypoint()` to snap the base position to the exact
lane centre before applying the lateral offset — eliminates drift from `fwd.y ≈ −0.007`.

---

## Current NPC Configuration (`simulatorAPI.py`)

```python
npc_offsets = [(40, 0.0)]   # 1 police car, 40 m ahead, ego lane centre
```

The police car is confirmed at **y ≈ −10.0** (ego lane centre). ✅

---

## Current Problem: Car Does NOT Change Lanes

The police car is in the correct lane. The ego vehicle drives **straight into it**
instead of swerving into the adjacent lane.

### Root Cause Hypothesis — Planner Path Spread Too Narrow

With `P_NUM_PATHS=7` and `P_GOAL_OFFSET=1.0 m`, the 7 spirals span only **±3 m**
around the goal (ego lane centre at y=−10). The widest avoidance spiral reaches
**y=−13**, which is exactly 3.0 m from the police car centre — right on the collision
boundary. During the curved approach to the police car, intermediate spiral points
pass even closer, so **all 7 paths are marked as colliding**. When every spiral
collides, the planner keeps the existing trajectory and never replans.

The adjacent lane centre at y=−13.5 is safely clear (3.5 m > 3.0 m threshold), but
no spiral reaches it when the goal is at y=−10 with only ±3 m spread.

---

## Candidate Fixes to Investigate

1. **Increase `P_GOAL_OFFSET`** in `planning_params.h` (e.g., `1.0 → 1.5` or `2.0`).  
   This widens the spiral fan so the outermost path clears the adj lane centre.  
   **Requires recompile** (`cd project/pid_controller && make`).

2. **Lateral nudge on NPC** — `npc_offsets = [(40, −0.5)]` keeps the car mostly in the
   lane but clears one side enough for a spiral to squeeze through without widening
   `P_GOAL_OFFSET`. Quick to test without recompile.

3. **Verify obstacle registration** — check for `[NPC0] frozen at x=... y=...` in logs.
   If missing, `have_obst` in `main.cpp` never becomes `true` and the planner is blind.

4. **Debug spiral selection** — add a `cout` in `path_planner()` (`main.cpp` ~line 155)
   to log how many of the 7 spirals are collision-free. Confirms the planner *sees* the
   obstacle vs. the issue being elsewhere.

5. **Check `_active_maneuver`** in `simulatorAPI.py` — if stuck at 0 (FOLLOW_LANE with
   no lane-change goal), the behavior FSM goal never shifts laterally, so even a wider
   spiral fan won't help without a goal offset.

---

## Test Suite Status

| Test file | Status | Notes |
|-----------|--------|-------|
| `project/test_avoidance_pid.py` | ✅ PASS | Standalone PID, synchronous mode |
| `project/test_avoidance_scenario.py` | ✅ PASS | |
| `project/test_lane_change_pid.py` | ✅ PASS | `POLICE_DIST_M=40`, `TRIGGER_DIST_M=25` |
| `project/test_slalom_pid.py` | ✅ PASS | All 5 criteria, direct lateral CTE |

> All tests use a standalone `SimplePID` (mirrors `pid_controller.cpp` exactly) and
> direct lateral CTE (`cte = loc.y - target_y`) — they do NOT call `run_main_pid.sh`.
> The failing behaviour is specific to the **live simulation** (`run_main_pid.sh`).

---

## Key Files

| File | Purpose |
|------|---------|
| `project/pid_controller/pid_controller.cpp` | **Student implementation** — P/I/D math |
| `project/pid_controller/main.cpp` | WebSocket loop, gains, steer/throttle error |
| `project/pid_controller/planning_params.h` | `P_NUM_PATHS`, `P_GOAL_OFFSET`, `CIRCLE_OFFSETS/RADII` |
| `project/pid_controller/cost_functions.cpp` | `collision_circles_cost_spiral` |
| `project/pid_controller/motion_planner.cpp` | Spiral generation + best-spiral selection |
| `project/simulatorAPI.py` | CARLA bridge, `SpawnNPC`, `get_waypoint`, NPC offsets |
| `project/test_slalom_pid.py` | Slalom test (PASSES) |
| `steer_pid_data.txt` / `throttle_pid_data.txt` | Live run PID logs |
| `project/plot_pid.py` | Matplotlib plot of PID logs |
