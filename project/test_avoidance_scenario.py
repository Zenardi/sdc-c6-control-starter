"""
test_avoidance_scenario.py
==========================
Senior AV Test Engineer — Deterministic CARLA PID Avoidance Test

Tests the PID controller's ability to avoid a stationary obstacle in Town03.

Architecture:
  - CARLA Synchronous Mode (dt=0.05s): physics is deterministic, no lag variation.
  - Standalone: reimplements the same PID math and CTE formula as main.cpp in
    Python so this test runs WITHOUT needing the C++ WebSocket binary.
  - The test validates the ALGORITHM (gains, CTE formula, output clamps), not
    the WebSocket transport.

Scenario:
  1. Spawn ego at a fixed straight-road transform in Town03.
  2. Spawn obstacle 50m directly ahead (along ego's forward vector).
  3. Drive ego toward obstacle at 30 km/h.
  4. At 20m distance: trigger avoidance (emergency brake + mild lane offset).
  5. Assert success criteria throughout.

Success Criteria:
  - Distance to obstacle NEVER reaches 0.0m (no collision).
  - CTE < 0.5m for > 95% of pre-avoidance frames (straight-line tracking).
  - Steering oscillation count ≤ 3 during avoidance maneuver.
  - Vehicle reaches stable state (speed < 0.5 m/s OR > 5m past obstacle)
    within 100 synchronous ticks after avoidance trigger.

Usage:
  # CARLA must be running first:
  #   ./project/CARLA/CarlaUE4.sh -RenderOffScreen -carla-world-port=2000
  #
  # Then:
  python3 project/test_avoidance_scenario.py
  # or with pytest:
  pytest project/test_avoidance_scenario.py -v

Dependencies: carla (Python package from CARLA install), math, time, logging
"""
import sys
import math
import time
import logging
import argparse
from dataclasses import dataclass, field
from typing import List, Optional

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
log = logging.getLogger("avoidance_test")

# ── Try to import CARLA Python bindings ───────────────────────────────────────
try:
    sys.path.insert(0, "CARLA/PythonAPI/carla/dist/carla-0.9.16-py3.11-linux-x86_64.egg")
    sys.path.insert(0, "CARLA/PythonAPI/carla/")
    import carla
except ImportError as e:
    log.error("Could not import CARLA Python bindings: %s", e)
    log.error("Run from the project/ directory: python3 test_avoidance_scenario.py")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# PID controller — Python mirror of pid_controller.cpp
# Same math as UpdateError / TotalError in pid_controller.cpp.
# ─────────────────────────────────────────────────────────────────────────────
class PID:
    """Python implementation of the C++ PID class (pid_controller.cpp)."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_max: float, out_min: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.out_min = out_max, out_min
        self._p_err = 0.0
        self._i_err = 0.0
        self._d_err = 0.0
        self._dt    = 0.05  # default first-frame dt

    def update_dt(self, dt: float):
        self._dt = max(dt, 1e-4)  # guard against zero

    def update_error(self, cte: float):
        prev = self._p_err
        self._p_err  = cte
        dt = min(self._dt, 0.5)  # cap dt (same as cpp: >0.5 → 0.5)
        self._d_err  = (cte - prev) / dt
        self._i_err += cte * dt

    def total_error(self) -> float:
        u = (-self.kp * self._p_err
             - self.ki * self._i_err
             - self.kd * self._d_err)
        return max(self.out_min, min(self.out_max, u))


# ─────────────────────────────────────────────────────────────────────────────
# Test configuration constants
# ─────────────────────────────────────────────────────────────────────────────
CARLA_HOST     = "localhost"
CARLA_PORT     = 2000
SYNC_DELTA     = 0.05          # s — fixed physics tick (deterministic)
TARGET_SPEED   = 30.0 / 3.6    # 30 km/h → m/s ≈ 8.33
OBSTACLE_DIST  = 50.0          # m — obstacle ahead of ego spawn
AVOIDANCE_DIST = 20.0          # m — trigger avoidance when closer than this
MAP_NAME       = "Town03"

# PID gains — same as main.cpp
STEER_KP, STEER_KI, STEER_KD       = 0.05, 0.0,   0.05
STEER_MAX, STEER_MIN                = 0.3,  -0.3
THROTTLE_KP, THROTTLE_KI, THROTTLE_KD = 0.10, 0.001, 0.05
THROTTLE_MAX, THROTTLE_MIN          = 1.0, -1.0

# Success / failure thresholds
MAX_CTE_STRAIGHT         = 0.5    # m  — CTE limit during straight-line approach
MAX_CTE_FRAC_VIOLATION   = 0.05   # fraction of frames allowed to violate the above
MAX_OSCILLATIONS         = 3      # max sign-reversals in steer_output during avoidance
MAX_TICKS_TO_STABLE      = 100    # ticks after avoidance trigger to reach stable state
STABLE_SPEED_THRESHOLD   = 0.5    # m/s — vehicle considered "stopped" below this
STABLE_CLEARANCE         = 5.0    # m — OR vehicle has passed obstacle by this much
MAX_RUN_TICKS            = 2000   # safety limit — end test after this many ticks


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────
def distance_2d(a: carla.Location, b: carla.Location) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def cte(veh_loc: carla.Location, veh_yaw_rad: float,
        wp_loc: carla.Location) -> float:
    """Cross-track error (lateral offset in vehicle frame).

    Mirrors the formula in main.cpp:
        dx = x_position - x_points[0]
        dy = y_position - y_points[0]
        error_steer = -sin(yaw)*dx + cos(yaw)*dy
    """
    dx = veh_loc.x - wp_loc.x
    dy = veh_loc.y - wp_loc.y
    return -math.sin(veh_yaw_rad) * dx + math.cos(veh_yaw_rad) * dy


def yaw_rad(transform: carla.Transform) -> float:
    return math.radians(transform.rotation.yaw)


# ─────────────────────────────────────────────────────────────────────────────
# Test result data class
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TestResult:
    passed: bool = True
    reason: str  = ""
    frames: int  = 0
    min_dist_to_obstacle: float = float("inf")
    max_cte_straight: float     = 0.0
    straight_cte_violations: int = 0
    straight_frames: int         = 0
    oscillation_count: int       = 0
    avoidance_triggered: bool    = False
    ticks_to_stable: Optional[int] = None
    cte_log:      List[float] = field(default_factory=list)
    speed_log:    List[float] = field(default_factory=list)
    steer_log:    List[float] = field(default_factory=list)
    distance_log: List[float] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# Main test
# ─────────────────────────────────────────────────────────────────────────────
def run_avoidance_test(host=CARLA_HOST, port=CARLA_PORT) -> TestResult:
    result = TestResult()
    client = None
    world  = None
    actors = []
    original_settings = None

    try:
        # ── Connect ──────────────────────────────────────────────────────────
        log.info("Connecting to CARLA at %s:%d …", host, port)
        client = carla.Client(host, port)
        client.set_timeout(30.0)
        world = client.get_world()

        # ── Load Town03 ───────────────────────────────────────────────────────
        current_map = world.get_map().name
        if "Town03" not in current_map:
            log.info("Loading %s (current: %s) …", MAP_NAME, current_map)
            world = client.load_world(MAP_NAME)
            time.sleep(5.0)

        # ── Enable synchronous mode ───────────────────────────────────────────
        original_settings = world.get_settings()
        sync_settings = carla.WorldSettings(
            synchronous_mode=True,
            fixed_delta_seconds=SYNC_DELTA,
            no_rendering_mode=True,   # headless: faster ticks
        )
        world.apply_settings(sync_settings)
        log.info("Synchronous mode ON (dt=%.3f s)", SYNC_DELTA)

        # ── Find a spawn point on a straight road ─────────────────────────────
        spawn_points = world.get_map().get_spawn_points()
        # Use spawn point 1 in Town03 (proven straight segment)
        ego_transform = spawn_points[1]
        log.info("Ego spawn: x=%.2f y=%.2f yaw=%.1f",
                 ego_transform.location.x, ego_transform.location.y,
                 ego_transform.rotation.yaw)

        # ── Compute obstacle transform (50m ahead along ego forward) ──────────
        ego_yaw = math.radians(ego_transform.rotation.yaw)
        fwd_x = math.cos(ego_yaw)
        fwd_y = math.sin(ego_yaw)
        obs_loc = carla.Location(
            x=ego_transform.location.x + OBSTACLE_DIST * fwd_x,
            y=ego_transform.location.y + OBSTACLE_DIST * fwd_y,
            z=ego_transform.location.z,
        )
        obs_transform = carla.Transform(obs_loc, ego_transform.rotation)
        log.info("Obstacle: x=%.2f y=%.2f (%.1fm ahead)",
                 obs_loc.x, obs_loc.y, OBSTACLE_DIST)

        # ── Spawn vehicles ────────────────────────────────────────────────────
        bp_lib = world.get_blueprint_library()
        ego_bp = bp_lib.find("vehicle.lincoln.mkz_2020")
        obs_bp = bp_lib.find("vehicle.tesla.model3")    # visually distinct obstacle

        ego = world.spawn_actor(ego_bp, ego_transform)
        actors.append(ego)
        obs = world.spawn_actor(obs_bp, obs_transform)
        obs.set_simulate_physics(False)   # stationary obstacle
        actors.append(obs)
        log.info("Spawned ego=%d, obstacle=%d", ego.id, obs.id)

        # Tick twice to let actors appear in world state
        world.tick(); world.tick()

        # ── PID controllers ───────────────────────────────────────────────────
        pid_steer    = PID(STEER_KP, STEER_KI, STEER_KD, STEER_MAX, STEER_MIN)
        pid_throttle = PID(THROTTLE_KP, THROTTLE_KI, THROTTLE_KD,
                           THROTTLE_MAX, THROTTLE_MIN)

        # ── Reference waypoint: use road waypoints 2m apart for path tracking ─
        carla_map = world.get_map()

        def get_reference_waypoint(loc: carla.Location):
            """Nearest road-centre waypoint (projected to driving lane)."""
            return carla_map.get_waypoint(
                loc, project_to_road=True,
                lane_type=carla.LaneType.Driving
            )

        # ── State machine ─────────────────────────────────────────────────────
        PHASE_APPROACH  = "APPROACH"    # normal driving toward obstacle
        PHASE_AVOIDANCE = "AVOIDANCE"   # emergency action triggered
        PHASE_STABLE    = "STABLE"      # car stopped / passed obstacle
        phase = PHASE_APPROACH
        avoidance_start_tick = None
        prev_steer = 0.0
        steer_sign_changes = 0
        prev_sim_time = -1.0

        log.info("─" * 60)
        log.info("Test running  (max %d ticks)", MAX_RUN_TICKS)
        log.info("%-8s %-8s %-8s %-8s %-12s %-10s",
                 "Tick", "Speed", "CTE", "Steer", "Dist2Obs", "Phase")
        log.info("─" * 60)

        for tick in range(MAX_RUN_TICKS):
            world.tick()
            result.frames += 1

            # ── Snapshot ──────────────────────────────────────────────────────
            ego_tf  = ego.get_transform()
            ego_vel = ego.get_velocity()
            speed   = math.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)
            dist    = distance_2d(ego_tf.location, obs_loc)
            veh_yaw = yaw_rad(ego_tf)

            # ── Reference waypoint for CTE ────────────────────────────────────
            wp = get_reference_waypoint(ego_tf.location)
            cte_val = cte(ego_tf.location, veh_yaw, wp.transform.location)

            # ── dt from fixed sync delta ──────────────────────────────────────
            sim_time = tick * SYNC_DELTA
            dt = SYNC_DELTA if prev_sim_time < 0 else sim_time - prev_sim_time
            prev_sim_time = sim_time
            pid_steer.update_dt(dt)
            pid_throttle.update_dt(dt)

            # ── Log ───────────────────────────────────────────────────────────
            result.cte_log.append(cte_val)
            result.speed_log.append(speed)
            result.distance_log.append(dist)
            result.min_dist_to_obstacle = min(result.min_dist_to_obstacle, dist)

            # ── FAILURE: collision (dist < 1.0m = vehicle bodies overlapping) ─
            if dist < 1.0:
                result.passed = False
                result.reason = f"COLLISION at tick {tick}: dist={dist:.2f}m"
                log.error("❌  %s", result.reason)
                break

            # ── Phase transitions ──────────────────────────────────────────────
            if phase == PHASE_APPROACH and dist <= AVOIDANCE_DIST:
                phase = PHASE_AVOIDANCE
                result.avoidance_triggered = True
                avoidance_start_tick = tick
                log.info("⚠️   Avoidance triggered at tick %d, dist=%.2fm", tick, dist)

            if phase == PHASE_AVOIDANCE and avoidance_start_tick is not None:
                ticks_since = tick - avoidance_start_tick
                past_obstacle = (ego_tf.location.x - obs_loc.x) * fwd_x + \
                                (ego_tf.location.y - obs_loc.y) * fwd_y
                if speed < STABLE_SPEED_THRESHOLD or past_obstacle > STABLE_CLEARANCE:
                    phase = PHASE_STABLE
                    result.ticks_to_stable = ticks_since
                    log.info("✅  Stable at tick %d (%d ticks after avoidance, speed=%.2f m/s)",
                             tick, ticks_since, speed)

            # ── Compute control ────────────────────────────────────────────────
            if phase == PHASE_APPROACH:
                # Normal PID tracking: follow road centre, target speed 30 km/h
                pid_steer.update_error(cte_val)
                steer = pid_steer.total_error()

                speed_err = speed - TARGET_SPEED
                pid_throttle.update_error(speed_err)
                throttle_raw = pid_throttle.total_error()
                throttle = max(0.0, throttle_raw)
                brake    = max(0.0, -throttle_raw)

            elif phase == PHASE_AVOIDANCE:
                # Emergency avoidance: full brake + slight left swerve (away from obstacle)
                # Steer 0.1 rad toward the free lane (south side of road in most Town03 spawns)
                steer    = -0.2    # constant mild swerve away
                throttle = 0.0
                brake    = 1.0     # full emergency brake
                # Override PID to prevent windup during emergency
                pid_steer.update_error(0.0)
                pid_throttle.update_error(0.0)

            else:  # STABLE
                steer    = 0.0
                throttle = 0.0
                brake    = 0.5

            result.steer_log.append(steer)

            # ── Count oscillations ────────────────────────────────────────────
            if phase == PHASE_AVOIDANCE:
                if steer * prev_steer < 0:   # sign reversal
                    steer_sign_changes += 1
            prev_steer = steer
            result.oscillation_count = steer_sign_changes

            # ── CTE violation tracking (straight approach only) ───────────────
            if phase == PHASE_APPROACH:
                result.straight_frames += 1
                if abs(cte_val) > MAX_CTE_STRAIGHT:
                    result.straight_cte_violations += 1
                result.max_cte_straight = max(result.max_cte_straight, abs(cte_val))

            # ── Apply control ─────────────────────────────────────────────────
            ego.apply_control(carla.VehicleControl(
                steer=float(steer),
                throttle=float(throttle),
                brake=float(brake),
            ))

            # ── Periodic log ──────────────────────────────────────────────────
            if tick % 20 == 0:
                log.info("%-8d %-8.2f %-8.3f %-8.3f %-12.2f %-10s",
                         tick, speed, cte_val, steer, dist, phase)

            # ── End conditions ────────────────────────────────────────────────
            if phase == PHASE_STABLE:
                log.info("Reached stable state — ending test at tick %d", tick)
                break

        # ── Post-run assertions ───────────────────────────────────────────────
        log.info("─" * 60)
        log.info("POST-RUN ASSERTIONS")
        log.info("─" * 60)

        # 1. No collision
        _assert(result, result.min_dist_to_obstacle >= 1.0,
                f"FAIL: Minimum distance to obstacle = {result.min_dist_to_obstacle:.2f}m "
                f"(required ≥ 1.0m)")
        log.info("✅  Min distance to obstacle: %.2fm (required ≥ 1.0m)",
                 result.min_dist_to_obstacle)

        # 2. CTE during straight approach
        if result.straight_frames > 0:
            violation_frac = result.straight_cte_violations / result.straight_frames
            _assert(result, violation_frac <= MAX_CTE_FRAC_VIOLATION,
                    f"FAIL: CTE exceeded {MAX_CTE_STRAIGHT}m in "
                    f"{violation_frac*100:.1f}% of straight-line frames "
                    f"(limit {MAX_CTE_FRAC_VIOLATION*100:.0f}%). "
                    f"Max CTE={result.max_cte_straight:.3f}m")
            log.info("✅  Straight-line CTE: max=%.3fm, violations=%.1f%%",
                     result.max_cte_straight, violation_frac * 100)

        # 3. Oscillation during avoidance
        _assert(result, result.oscillation_count <= MAX_OSCILLATIONS,
                f"FAIL: Steering oscillations = {result.oscillation_count} "
                f"(limit {MAX_OSCILLATIONS})")
        log.info("✅  Steering oscillations: %d (limit %d)",
                 result.oscillation_count, MAX_OSCILLATIONS)

        # 4. Avoidance was triggered
        _assert(result, result.avoidance_triggered,
                "FAIL: Avoidance was never triggered — obstacle may not have been in path")
        log.info("✅  Avoidance maneuver triggered")

        # 5. Vehicle reached stable state within time limit
        if result.ticks_to_stable is not None:
            _assert(result, result.ticks_to_stable <= MAX_TICKS_TO_STABLE,
                    f"FAIL: Vehicle did not stabilise within {MAX_TICKS_TO_STABLE} ticks "
                    f"(took {result.ticks_to_stable})")
            stable_time = result.ticks_to_stable * SYNC_DELTA
            log.info("✅  Stable in %.1fs / %d ticks (limit %d ticks / %.0fs)",
                     stable_time, result.ticks_to_stable,
                     MAX_TICKS_TO_STABLE, MAX_TICKS_TO_STABLE * SYNC_DELTA)
        else:
            _assert(result, False,
                    "FAIL: Vehicle never reached stable state within test horizon")

    except Exception as exc:
        result.passed = False
        result.reason = f"Exception: {exc}"
        log.exception("Unexpected error during test")

    finally:
        # ── Cleanup ────────────────────────────────────────────────────────────
        log.info("Cleaning up %d actors …", len(actors))
        client_local = client
        if client_local is not None and world is not None:
            if original_settings is not None:
                world.apply_settings(original_settings)
                log.info("Synchronous mode restored to original settings")
            for actor in actors:
                try:
                    actor.destroy()
                except Exception:
                    pass
            world.tick()   # flush destroy commands
        log.info("Cleanup complete")

    return result


def _assert(result: TestResult, condition: bool, message: str):
    """Non-throwing assertion: mark result as failed and log, but continue checking."""
    if not condition:
        result.passed = False
        if not result.reason:
            result.reason = message
        log.error("❌  %s", message)


# ─────────────────────────────────────────────────────────────────────────────
# pytest entry-point
# ─────────────────────────────────────────────────────────────────────────────
def test_pid_obstacle_avoidance():
    """pytest wrapper — skip if CARLA is not reachable."""
    import socket
    try:
        s = socket.create_connection((CARLA_HOST, CARLA_PORT), timeout=3)
        s.close()
    except (ConnectionRefusedError, OSError):
        import pytest
        pytest.skip("CARLA not running — start CarlaUE4 before running this test")

    result = run_avoidance_test()
    _print_summary(result)
    assert result.passed, result.reason


# ─────────────────────────────────────────────────────────────────────────────
# CLI entry-point
# ─────────────────────────────────────────────────────────────────────────────
def _print_summary(result: TestResult):
    log.info("=" * 60)
    log.info("TEST SUMMARY")
    log.info("=" * 60)
    log.info("Result         : %s", "PASS ✅" if result.passed else "FAIL ❌")
    if not result.passed:
        log.info("Reason         : %s", result.reason)
    log.info("Total ticks    : %d  (%.1fs sim time)",
             result.frames, result.frames * SYNC_DELTA)
    log.info("Min dist obst  : %.2f m", result.min_dist_to_obstacle)
    log.info("Max CTE (appr) : %.3f m", result.max_cte_straight)
    log.info("Oscillations   : %d", result.oscillation_count)
    log.info("Ticks to stable: %s", str(result.ticks_to_stable))
    log.info("Avoidance trig : %s", result.avoidance_triggered)
    log.info("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CARLA PID Avoidance Integration Test")
    parser.add_argument("--host", default=CARLA_HOST)
    parser.add_argument("--port", type=int, default=CARLA_PORT)
    args_parsed = parser.parse_args()

    result = run_avoidance_test(args_parsed.host, args_parsed.port)
    _print_summary(result)
    sys.exit(0 if result.passed else 1)
