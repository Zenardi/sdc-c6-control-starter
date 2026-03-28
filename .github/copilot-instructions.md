# Copilot Instructions

## Project Overview

Udacity Self-Driving Car Nanodegree — **C6: Control and Trajectory Tracking**. Students implement a PID controller in C++ that steers and throttles a vehicle to follow a pre-planned trajectory inside the CARLA simulator. The planning stack (behavior FSM, motion planner, spiral generation, velocity profiling) is already provided; the student's task is `pid_controller.h/.cpp`.

## Build & Run

All commands run from `project/pid_controller/`:

```bash
# Build
cmake .
make

# Run (from project/)
./run_main_pid.sh   # starts pid_controller, then simulatorAPI.py
```

CARLA must be running at `project/CARLA/` before launching. The simulator listens on the WebSocket port that `main.cpp` connects to.

After each C++ edit, only `make` is needed (no need to re-run `cmake .`).

## Architecture

```
CARLA Simulator (Python simulatorAPI.py)
        ↕  WebSocket (uWS, main.cpp)
┌─────────────────────────────────────┐
│  BehaviorPlannerFSM                 │  ← high-level state: FOLLOW_LANE /
│    └─ MotionPlanner                 │     FOLLOW_VEHICLE / DECEL_TO_STOP / STOPPED
│         ├─ CubicSpiral (paths)      │
│         ├─ CostFunctions (select)   │
│         └─ VelocityProfileGenerator │
│  PID (steering)   ← CTE             │  ← **student implements this**
│  PID (throttle)   ← velocity error  │
└─────────────────────────────────────┘
        ↕  steering / throttle / brake
     CARLA vehicle actuators
```

`main.cpp` drives the loop: receives sensor/waypoint data over WebSocket → runs behavior + motion planner → feeds the resulting trajectory to the two PID instances → sends control outputs back.

PID output data is written to `steer_pid_data.txt` and `throttle_pid_data.txt` for post-run analysis with `plot_pid.py`.

## Student Implementation Target

`pid_controller.h` / `pid_controller.cpp` — the `PID` class must implement:

| Method | Signature | Purpose |
|--------|-----------|---------|
| `Init` | `(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min)` | Store gains and output clamp values |
| `UpdateError` | `(double cte)` | Update P/I/D error terms using `delta_time` |
| `TotalError` | `() → double` | Return `Kp*p_error + Ki*i_error + Kd*d_error`, clamped to limits |
| `UpdateDeltaTime` | `(double new_delta_time) → double` | Store and return dt |

Two PID instances are created in `main.cpp`: one for **steering** (input: cross-track error) and one for **throttle/brake** (input: velocity error).

## Key Conventions

- **Naming**: `PascalCase` classes, `snake_case` functions, `_underscore_prefix` member variables, `P_UPPER_SNAKE` planning constants.
- **Headers**: `#pragma once`; CARLA types aliased via `namespace cg = carla::geom`.
- **Structs**: Core types (`State`, `PathPoint`, `TrajectoryPoint`, `MPC_State`) live in `structs.h`. Planning constants (`P_NUM_PATHS`, `P_SPEED_LIMIT`, etc.) live in `planning_params.h`.
- **Logging**: Use `LOG(INFO)` / `LOG(WARNING)` (glog), not `std::cout`, for runtime diagnostics.
- **File I/O for PID data**: `main.cpp` writes CSV lines directly to `steer_pid_data.txt` and `throttle_pid_data.txt`; don't add extra file handles.

## Dependencies

| Dependency | Location | Notes |
|------------|----------|-------|
| CARLA 0.9.9/0.9.10 | `/opt/carla-simulator/`, headers at `/opt/carla-source/` | Must be installed; not bundled |
| Eigen 3.3.7 | `project/pid_controller/eigen-3.3.7/` | Header-only, bundled |
| uWebSockets | `/usr/local/` | System install via `install-ubuntu.sh` |
| Boost 1.72.0 | CARLA build dir | Linked as static `.a` from CARLA's build tree |

C++ standard: **C++14** (`-std=c++14 -O3`).


## Implementation Details (Refined)

The `PID` class must handle real-world physical constraints to be "Gold" status:

- **Integral Anti-Windup**: Implement a limit for the integral term. If the `TotalError` exceeds `output_lim_max/min`, stop accumulating `i_error` to prevent "overshoot" after long turns.
- **Derivative Smoothing**: Use a small low-pass filter or simple averaging for `d_error` if the `cte` from CARLA is noisy.
- **Delta Time Stability**: Always multiply `i_error` by `dt` and divide `d_error` by `dt`. If `dt == 0`, skip the update to avoid division by zero.

## Simulation-Driven Development (SDD) Protocol

Copilot should prioritize "Testing-First" logic:

1. **Unit Test First**: Before implementing in C++, generate a Python/C++ mock test to verify the math of the PID (e.g., "If error is constant 1.0, does the output increase linearly?").
2. **Synchronous Mode**: Always assume CARLA is in Synchronous Mode (fixed `dt`) to ensure deterministic PID behavior.
3. **The "Gold" Success Criteria**:
    - **No Oscillations**: The vehicle must settle into a lane change within 2 "wobbles."
    - **CTE Tolerance**: Cross-Track Error must stay below 0.5m during steady-state cruising.
    - **Zero Collisions**: Absolute failure if any `sensor.other.collision` event triggers.

## Testing Scenarios (Verification)

When asked to "verify" or "test," use these predefined scenarios:

| Scenario | Setup | Goal |
| :--- | :--- | :--- |
| **Straight Speed** | No obstacles, target 30km/h | Zero throttle oscillation. |
| **The Police Slalom**| Staggered police cars in lanes (offsets +/- 1.0m) | Forced lane changes; check damping. |
| **Emergency Brake** | Lead vehicle brakes at 10m/s² | Test PID reaction time and windup recovery. |

## Tuning Heuristics for Copilot

When providing tuning advice, use the following priority:
1. **P (Proportional)**: Increase until the car starts to oscillate, then cut by 50%.
2. **D (Derivative)**: Increase to "damp" the P-oscillation. This is critical for high-speed stability in Town03.
3. **I (Integral)**: Use very sparingly (often < 10% of P) only to correct long-term drift in curves.