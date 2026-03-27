# PID Controller — Engineering Troubleshooting Report

> **Project:** Control and Trajectory Tracking for Autonomous Vehicle  
> **Course:** Udacity Self-Driving Car Nanodegree — C6 Control  
> **Environment:** CARLA 0.9.16 · Town06_Opt · Ubuntu · Python 3.11 · C++17  
> **Outcome:** Vehicle drives **700 m** through Town06_Opt, successfully passing all 3 parked NPCs with FSM in `FOLLOW_LANE` (no `DECEL_TO_STOP` required), collecting **10,552 rows** of PID telemetry

---

## Overview

This document chronicles every bug, root cause, and fix encountered while implementing and running the PID controller project in a local environment that differs significantly from the Udacity Workspace the project was designed for. Issues ranged from build system failures and Python runtime incompatibilities to subtle timing bugs in the control loop that caused the vehicle to shoot off-road at 14 m/s.

---

## Phase 1 — Build System: No CARLA Headers

### Problem

The project's `CMakeLists.txt` expected CARLA C++ source headers at `/opt/carla-source/` and a `libcarla_client.a` static library. The local environment has only a **precompiled CARLA 0.9.16 binary** at `project/CARLA/` — no C++ headers, no source tree.

```
fatal error: carla/geom/Transform.h: No such file or directory
```

### Diagnosis

The C++ controller (`main.cpp` and the planning stack) includes several CARLA geometry and client headers, but it **never actually calls the CARLA C++ API at runtime**. The architecture is:

```
CARLA Simulator (Python) ──WebSocket──▶ C++ pid_controller
```

The C++ binary speaks WebSocket to the Python bridge (`simulatorAPI.py`). CARLA types are only used for geometry math. The headers just need to **compile** — not link against a real CARLA library.

### Fix

1. Created **minimal CARLA stub headers** at `project/pid_controller/carla/` covering every type the planning stack references:

   | Header | Provides |
   |--------|----------|
   | `geom/Vector3D.h` | `struct` with `x,y,z`, `Length()`, `operator+/-` |
   | `geom/Location.h` | Inherits `Vector3D` (needed for subtraction in motion planner) |
   | `geom/Rotation.h` | `pitch`, `yaw`, `roll` |
   | `geom/Transform.h` | `Location` + `Rotation` |
   | `road/RoadTypes.h` | `using JuncId = int32_t` |
   | `client/Waypoint.h` | `GetTransform()`, `GetNext()`, `IsJunction()`, `GetJunctionId()` |
   | `client/Map.h` | `GetWaypoint(Location)` |
   | Other client/image/sensor | Empty `#pragma once` stubs |

2. Rewrote `CMakeLists.txt`:
   - C++17 (from C++14 — needed by student code patterns)
   - System Boost 1.88 at `/usr/lib/x86_64-linux-gnu/`
   - `librpc.a` from `project/CARLA/CarlaUE4/Plugins/Carla/CarlaDependencies/lib/`
   - Removed all `/opt/carla-source` references
   - Added `${CMAKE_CURRENT_SOURCE_DIR}` as include root for stub headers

**Build iteration 1:** `operator-` missing on `Vector3D` — added it.  
**Build iteration 2:** `std::uint32_t` undefined — added `#include <cstdint>` to `Vector3D.h`.  
**Result:** Clean build — `[100%] Built target pid_controller`.

---

## Phase 2 — Python 3.11 Asyncio Incompatibility

### Problem

`simulatorAPI.py` was written for Python 3.6–3.9 and used the deprecated `@asyncio.coroutine` decorator and `yield from` syntax, both of which were **removed in Python 3.11**:

```
AttributeError: module 'asyncio' has no attribute 'coroutine'
```

A secondary issue: `asyncio.wait([list])` with a plain list was deprecated and `asyncio.run()` requires a coroutine, not a `Future` object.

### Fix

- `@asyncio.coroutine` + `yield from` → `async def` + `await`
- `asyncio.wait([list])` → wrapped in `async def _run(): await asyncio.gather(...)` and called via `asyncio.run(_run())`

---

## Phase 3 — WebSocket Segfault on Disconnect

### Problem

Every time Python disconnected (crash or clean exit), the C++ process died with a segfault:

```
Segmentation fault (core dumped)
```

### Root Cause

The `onDisconnection` handler in `main.cpp` called `ws.close()`. In uWS, the socket is **already closed** at the point `onDisconnection` fires — calling `ws.close()` again is a double-close that corrupts internal uWS state and crashes the process.

### Fix

Removed the `ws.close()` call from the `onDisconnection` handler entirely.

---

## Phase 4 — Throttle Error Sign Inversion

### Problem

Vehicle was **braking when it should accelerate** and vice versa. Throttle output was negative (braking) when the vehicle was slower than the target speed.

### Root Cause

The throttle error was initially computed as:
```cpp
error_throttle = v_points.back() - velocity;  // desired - actual
```

But `TotalError()` already applies a **negative sign convention**: `-(Kp·p + Ki·i + Kd·d)`. With desired > actual (vehicle too slow), `error > 0`, so `TotalError = -Kp·(+error) < 0` → brake output. The opposite of what was needed.

### Fix

```cpp
error_throttle = velocity - v_points[v_points.size() - 1];  // actual - desired
```

Now when vehicle is too slow: `error < 0`, `TotalError = -Kp·(-error) > 0` → throttle ✓

---

## Phase 5 — "No Spirals Generated" Every Frame

### Problem

After all the above fixes, the simulation ran but the vehicle never moved. Every frame printed:

```
Error: No spirals generated. ego=(-103.179,-14.4349) yaw=0
```

Despite the vehicle correctly spawning at `(-103.2, -14.4)`, the yaw was always reported as `0` (east) even though the vehicle was physically facing south (yaw ≈ -89.4° = -1.56 rad).

### Root Cause — Wrong Spawn Point (spawn[1])

The original spawn point `spawn_points[1]` placed the vehicle **8 meters from a junction**. The behavior planner FSM immediately entered `DECEL_TO_STOP` state. The spiral generator (Newton-Raphson cubic spiral fitting) fails when the goal is geometrically complex (stop line at junction at extreme angle).

**Fix:** Changed to `spawn_points[4]` — a straight southbound road with no junction within 30m.

### Root Cause — Stale Yaw on First Frame

Even after fixing the spawn point, yaw was still reported as `0`. Debug output revealed:

```
ego=(-103.179,-14.4349) yaw=0  goal_set.size=7  goal0=(-103.x,-22.x)
```

Tracing the code:

1. `simulatorAPI.py` has a global `_prev_yaw = 0` (line 83)
2. `world.move()` updates `_prev_yaw` only when `len(way_points) > 1`
3. On the **very first frame**, `way_points` has only 1 entry → `world.move()` never updates `_prev_yaw`
4. Python sends `'yaw': _prev_yaw = 0` (east) to C++
5. C++ `path_planner()` only set `ego_state.rotation.yaw = yaw` inside the `if (x_points.size() > 1)` block
6. On the first frame, `x_points.size() == 1` → the block never executes → yaw stays at default `0`

With yaw=0 (facing east), the goal 8m to the south appears **behind and to the right** of the vehicle in ego frame. The spiral generator cannot produce valid cubic spirals to goals behind the start point.

**Fixes applied:**

In `simulatorAPI.py` — seed `_prev_yaw` immediately when the vehicle is spawned:
```python
_prev_yaw = player.get_transform().rotation.yaw * math.pi / 180
```

In `main.cpp` — apply received yaw even on the first frame:
```cpp
if (x_points.size() > 1) {
    // ... compute yaw from trajectory ...
} else {
    ego_state.rotation.yaw = yaw;  // ← added this else clause
}
```

---

## Phase 6 — Process Management: Race Condition and Orphaned Actors

### Problem A: Python crashes with `ConnectionRefusedError`

`simulatorAPI.py` line 99 opens the WebSocket connection at **module load time** (not inside a function). If the C++ server is not already listening on port 4567, Python dies immediately:

```
ConnectionRefusedError: [Errno 111] Connection refused
```

### Fix A

Always start the C++ server first and wait 2–3 seconds before starting Python:
```bash
nohup ./pid_controller/pid_controller > /tmp/pid.log 2>&1 & disown $!
sleep 3
nohup python3 -u simulatorAPI.py > /tmp/sim.log 2>&1 & disown $!
```

Using `nohup ... & disown` (not just `&`) is critical — bare `&` processes are killed when the shell session ends.

### Problem B: Orphaned sensor actors blocking respawn

After a Python crash, CARLA's collision sensor and lane invasion sensor actors are **not destroyed** along with the vehicle. They linger at spawn[4] and cause `spawn_actor()` to fail silently — Python enters an infinite `while self.player is None:` loop at 40% CPU with no output.

### Fix B

Before every run, destroy all vehicle and sensor actors in the CARLA world:
```python
import carla
c = carla.Client('localhost', 2000); c.set_timeout(5)
w = c.get_world()
to_kill = [a.id for a in w.get_actors()
           if 'vehicle' in a.type_id or 'sensor' in a.type_id]
c.apply_batch([carla.command.DestroyActor(i) for i in to_kill])
```

---

## Phase 7 — Delta Time Bug: Integral Windup at 60x Speed

### Problem

After the yaw fix, the vehicle spawned and spirals generated — but the vehicle still shot off-road, reaching 14–16 m/s before any corrective update arrived. The throttle integral was accumulating far too fast.

### Root Cause

The original `main.cpp` computed delta time using `time_t` / `difftime()`:

```cpp
time(&timer);
new_delta_time = difftime(timer, prev_timer);
```

`difftime()` has **1-second resolution**. CARLA sends messages at ~60Hz. Every sub-second message sees `difftime = 0`, triggering the guard:

```cpp
new_delta_time = (new_delta_time > 0) ? new_delta_time : 1.0;
```

This means **every message was processed as if 1 full second had elapsed**. At 60Hz, the integral accumulated 60× faster than intended. A `Ki=0.05` term behaved as if it were `Ki=3.0`.

### Fix

Replaced the coarse wall-clock timer with the **CARLA simulation clock** from the JSON payload:

```cpp
double sim_time = data["time_step"];  // simulation seconds from start
double new_delta_time = sim_time - prev_sim_time;
prev_sim_time = sim_time;
// Guard: use 0.05s (one physics frame) on first message
if (new_delta_time <= 0) new_delta_time = 0.05;
```

This gives sub-millisecond dt accuracy aligned with the actual physics tick rate.

---

## Phase 8 — Throttle Overshoot: Open-Loop Interval

### Problem

Even with correct delta time, the vehicle briefly tracked the trajectory then overshot again. Analysis of the PID data:

- Rows 1–39: good behavior — throttle 0.1–0.4, speed rising toward target
- Row 40+: speed jumps to 14 m/s, vehicle leaves road, stuck

### Root Cause

The C++ controller and Python bridge are **not tightly coupled**. Python only requests a new control update from C++ when `len(way_points) < update_point_thresh` (was `16`). Trajectory waypoints are consumed at a fake rate of `velocity × 0.05 m/step × 60 steps/second = 9 m/s`. A 20-point trajectory is exhausted in ~1 second.

During that 1-second open-loop interval, Python applies the **last received throttle command** (`≈0.9`) at 60 FPS to the CARLA physics engine. The Lincoln MKZ, with no speed limiter, accelerates from 0 to 14+ m/s in one second.

The vehicle's physical position and the mathematical trajectory diverge completely — the planner's reference path is still on the straight road while the physics vehicle is off-road at high speed.

### Fix — Three-Part

**1. Lower `Kp_throttle`** (0.3 → 0.15):  
Max initial throttle output drops from 0.9 to 0.45 for a 3 m/s error. The open-loop acceleration is halved.

**2. Hard-cap throttle output** (1.0 → 0.6):  
Even if gains are high, the actuator is clamped. Lincoln MKZ with throttle=0.6 takes ~3–4 seconds to reach 3 m/s instead of < 1 second.

**3. Increase update frequency** (`update_point_thresh` 16 → 4):  
Python requests a new control frame from C++ 4× more often (~every 250ms instead of ~1s). The open-loop window is cut to a quarter, dramatically reducing how much the vehicle can drift from the target before a correction arrives.

**4. Add derivative term to throttle** (Kd=0 → 0.1):  
Provides velocity-change damping. When the vehicle is accelerating rapidly, the positive `d_error/dt` produces a negative Kd contribution that reduces throttle before the speed overshoots.

**Final throttle gains:**
```cpp
pid_throttle.Init(0.15, 0.001, 0.1, 0.6, -1.0);
//               Kp    Ki     Kd   max  min
```

---

## Phase 9 — Resubmission: NPC Avoidance Failures (Root Cause Analysis)

After the first submission was rejected ("car cannot cross first obstacle"), three compounding bugs were identified and fixed:

### Bug 1 — Wrong Map: Town10HD_Opt → Town06_Opt

The initial runs used Town10HD_Opt (a city map with intersections). The reviewer's scenario requires a **long straight highway** with 3 parked NPCs spaced 30 / 65 / 110 m ahead of the spawn. Town10HD_Opt spawn[4] faces south (yaw ≈ −90°), so the `SpawnNPC` formula (which offsets by `forward.x`) placed all 3 NPCs laterally beside the vehicle instead of ahead. The vehicle never encountered any obstacle.

**Fix:** Load `Town06_Opt` (long straight highway) in `simulatorAPI.py` before creating the world.

### Bug 2 — NPC Spawn Anchor Mismatch

`SpawnNPC` anchored its 30/65/110 m offsets off `spawn_points[1]` while the vehicle spawned at `spawn_points[4]`. Even after fixing the map, the NPCs appeared in a different section of the road.

**Fix:** Change `SpawnNPC` anchor from `spawn_points[1]` to `spawn_points[4]`.

### Bug 3 — Obstacle `rotation.yaw` Defaulted to 0 (East-Facing)

In `set_obst()`, the obstacle struct's `rotation.yaw` was never assigned — it defaulted to `0.0` (east). The NPCs face west (`yaw = π`). The collision circle placement formula is:

```
circle_x = obs.x + OFFSET * cos(obs.yaw)
```

With `yaw=0`: `OFFSET[+3]` placed a circle 3 m **east** of NPC1 (at x=573.9), instead of 3 m west (correct: x=567.9). A 20 m spiral ending at x=576 appeared to collide with the phantom circle at x=573.9, even though the vehicle was 25 m from the NPC. The planner then selected the maximum north detour path, creating a violent steering demand.

**Fix:** `obstacle.rotation.yaw = M_PI` in `set_obst()`.

### Bug 4 — Derivative Kick at High Simulation Rate

The reference Kd=0.25 was tuned for the Udacity VM at ~10 Hz (dt ≈ 0.1 s). CARLA 0.9.16 runs at ~20 Hz (dt ≈ 0.05 s). At 20 Hz, the effective derivative gain doubles: `Kd/dt = 0.25/0.05 = 5.0`. A 0.06 m CTE change per frame saturated the ±0.3 steering output. Combined with the `if (velocity < 0.5) pid.UpdateError(0.0)` guard (which set p_error=0 while prev_error was large), this produced d_error = (0 − prev_error)/dt → full steer saturation → immediate spin.

**Fix:** Reduce Kd from 0.25 to 0.05. Add `if (dt > 0.5) dt = 0.5` cap in `UpdateError()` to prevent Kd explosion on slow/stalled CARLA frames.

### Bug 5 — Vehicle-Position vs. Trajectory-Tip Ego State

When the vehicle detoured north to avoid an NPC, a new spiral was planned starting from the vehicle's actual position (off center). But the old trajectory waypoints were still being tracked from road center. The CTE reference jumped on the next planning cycle, creating artificial CTE spikes that triggered additional derivative kicks even with Kd=0.05.

**Fix:** Reverted to reference-style planning: use `x_points[last] / y_points[last]` (trajectory tip) as the ego state for spiral generation, so new spirals continue smoothly from the current trajectory end rather than from the vehicle's instantaneous position.

---

## Phase 9 — Final Verified Run

After all fixes, the vehicle:

- Spawned at spawn[4] in Town06_Opt (x=600.9, y=−9.96) facing west (yaw ≈ −180°)
- Generated spirals on the first frame and began tracking immediately
- Drove west through the NPC zone (x=570–490) with FSM **staying at FOLLOW_LANE (behavior=0)**
- **Passed NPC0 at x=570.9** — motion planner selected a collision-free spiral around it
- **Passed NPC1 at x=535.9** — motion planner selected a collision-free spiral around it
- **Passed NPC2 at x=490.9** — motion planner selected a collision-free spiral around it
- Continued west for 700+ m total, reaching x ≈ −108 before the run was halted
- Collected **10,552 rows** of PID telemetry
- Steer mean CTE = **0.043 m** (std = 0.290 m) — excellent tracking throughout

The key insight that unlocked this success: using `x_points[last] / y_points[last]` (trajectory tip) as the spiral-generation ego state, so the planner always sees the NPCs from *ahead* of them rather than from behind (which caused all 7 candidate spirals to collide with the NPC).

---

## Final PID Gain Summary

| Controller | Kp | Ki | Kd | Output limits |
|------------|-----|------|------|--------------|
| Steering   | 0.05 | 0.0 | 0.05 | [−0.3, 0.3] |
| Throttle   | 0.10 | 0.001 | 0.05 | [−1.0, 1.0] |

**Steering design rationale:**
- Kd reduced to match 20 Hz simulation rate (Kd/dt = 0.05/0.05 = 1.0; same effective gain as reference 0.25 at 10 Hz)
- Ki=0: integral windup through NPC detour maneuvers causes latent oversteer after returning to lane
- Output clamped to ±0.3: sufficient for 0.5 m lateral detours on a straight highway

**Throttle design rationale:**
- Kp=0.10: conservative speed correction; avoids wheel-spin overshoot from rest
- Ki=0.001: small integral corrects steady-state under-speed without windup risk
- Kd=0.05: small derivative damps velocity surges at replanning transitions

---

## Lessons Learned

1. **Simulator clock ≠ wall clock.** Always use the simulation timestamp for dt — `time()` resolution is too coarse for any sub-second control loop.

2. **Open-loop intervals are dangerous.** When a controller runs infrequently relative to the physics engine, the last command is applied hundreds of times before a correction arrives. Tune gains conservatively and cap outputs to account for this.

3. **First-frame initialization matters.** Any state variable (like ego yaw) that is computed from a history of at least N frames must be explicitly seeded on the first frame. A wrong initial condition can make the entire planning stack fail silently.

4. **Stub headers are a valid build strategy.** When a library is only needed for type definitions (not runtime calls), minimal stubs that compile without linking are sufficient and save significant setup effort.

5. **Process lifecycle in WebSocket architectures.** Server must be ready before client starts. Detach both processes with `nohup + disown`. Clean up all child actors after crashes — simulators leave ghost objects that block future spawns.

6. **Struct field defaults matter.** Leaving `obstacle.rotation.yaw` uninitialized defaulted to 0 (east-facing) — silently placing collision circles 6 m from where the NPC actually was. Always explicitly set all fields that participate in geometric calculations.

7. **Gain scaling depends on simulation rate.** A derivative gain `Kd` tuned for 10 Hz is 2× too aggressive at 20 Hz. When deploying on hardware or a different simulator version, always verify `Kd/dt` remains in the intended range, or normalize by `dt` in the controller formula.

8. **Ego state planning origin matters.** Planning spirals from the vehicle's instantaneous (off-path) position creates CTE jumps in the trajectory buffer. Always plan from the trajectory tip (last buffered waypoint) to maintain continuity between planning cycles.
