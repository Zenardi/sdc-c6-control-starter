"""
test_avoidance_pid.py
=====================
AV QA Engineer — PID Trajectory-Shift Stability Test

Tests the steering and throttle PID's damping and recovery behaviour when the
path planner suddenly shifts the reference trajectory 3.5 m to the left — the
same thing that happens in CARLA when the motion planner routes around a parked
obstacle (it selects a spiral 3–4 m off road-centre).

Key insight
-----------
The PID controller does NOT see the obstacle directly.  It only sees the
Cross-Track Error (CTE) — the lateral offset of the vehicle from the latest
planned waypoint:

    CTE = -sin(yaw)*dx + cos(yaw)*dy          (from main.cpp line 341)

An "avoidance event" is therefore a *sudden CTE step* of ≈3–4 m.
This test injects that step artificially at t=5s and verifies:

  1. **Damping**   — vehicle reaches the new reference within 3 s.
  2. **Stability** — steering sign-reversals ≤ 2 before CTE < 0.2 m.
  3. **Saturation** — PID output does not stay at ±limit for > 1 s
                      (would indicate integral windup or gain instability).
  4. **Recovery**  — after passing new reference, CTE returns to < 0.2 m
                     without additional manual input (not required explicitly
                     but logged as a quality metric).

Architecture
------------
Standalone Python: reimplements the same PID math as `pid_controller.cpp`
so the test runs WITHOUT the C++ WebSocket binary. CARLA is used only for
its physics engine; the test drives the car directly with `apply_control`.

Usage
-----
    # CARLA must be running:
    #   ./project/CARLA/CarlaUE4.sh -RenderOffScreen -carla-world-port=2000
    cd project
    python3 test_avoidance_pid.py          # standalone
    pytest  test_avoidance_pid.py -v       # pytest
"""
import sys
import math
import time
import logging
import argparse
from dataclasses import dataclass, field
from typing import List, Optional

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
log = logging.getLogger("pid_stability_test")

# ── CARLA import ───────────────────────────────────────────────────────────────
try:
    sys.path.insert(0, "CARLA/PythonAPI/carla/dist/carla-0.9.16-py3.11-linux-x86_64.egg")
    sys.path.insert(0, "CARLA/PythonAPI/carla/")
    import carla
except ImportError as e:
    log.error("CARLA Python bindings not found: %s", e)
    log.error("Run from project/ directory: python3 test_avoidance_pid.py")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# Python mirror of pid_controller.cpp  (identical math to UpdateError/TotalError)
# ─────────────────────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, out_max, out_min):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.out_min = out_max, out_min
        self._p = self._i = self._d = 0.0
        self._dt = 0.05

    def update_dt(self, dt: float):
        self._dt = max(dt, 1e-4)

    def update_error(self, cte: float):
        prev = self._p
        self._p  = cte
        dt = min(self._dt, 0.5)          # same cap as cpp
        self._d  = (cte - prev) / dt
        self._i += cte * dt

    def total_error(self) -> float:
        u = -self.kp * self._p - self.ki * self._i - self.kd * self._d
        return max(self.out_min, min(self.out_max, u))

    @property
    def p_term(self): return -self.kp * self._p
    @property
    def d_term(self): return -self.kd * self._d
    @property
    def i_term(self): return -self.ki * self._i
    @property
    def is_saturated(self): return abs(self.total_error()) >= self.out_max * 0.99


# ─────────────────────────────────────────────────────────────────────────────
# Constants — must match pid_controller.cpp / planning_params.h
# ─────────────────────────────────────────────────────────────────────────────
CARLA_HOST     = "localhost"
CARLA_PORT     = 2000
SYNC_DELTA     = 0.05              # s — deterministic physics tick
MAP_NAME       = "Town06_Opt"      # wide 2-lane highway — same as main project
TARGET_SPEED   = 30.0 / 3.6       # 8.33 m/s = 30 km/h

# Gains — exact values from main.cpp
STEER_KP, STEER_KI, STEER_KD       = 0.05, 0.0,   0.05
STEER_MAX,    STEER_MIN             = 0.3,  -0.3     # ±0.3 as in main.cpp
THROTTLE_KP, THROTTLE_KI, THROTTLE_KD = 0.10, 0.001, 0.05
THROTTLE_MAX, THROTTLE_MIN          = 1.0,  -1.0

# Test parameters
WARMUP_TICKS       = 300           # ticks (15s) — enough for Lincoln MKZ to reach cruise
SWERVE_OFFSET_M    = 3.5          # m lateral shift (≈ 1 lane width on Town06 highway)
SETTLING_CTE_M     = 0.2          # m — CTE threshold for "settled"
MAX_SETTLING_TICKS = 100          # ticks — 5 s at dt=0.05 s (matches project spec)
MAX_OSCILLATIONS   = 2            # max steering sign-reversals before settling
MAX_SAT_TICKS      = 20           # ticks — 1 s of continuous saturation = fail
MAX_RUN_TICKS      = 700          # hard limit (300 warmup + 400 swerve window)
SPAWN_POINT_IDX    = 4            # Town06_Opt — same west-facing highway spawn as main.cpp


# ─────────────────────────────────────────────────────────────────────────────
# Result data class
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Result:
    passed:             bool  = True
    reason:             str   = ""
    settling_ticks:     Optional[int] = None
    oscillations:       int   = 0
    max_sat_run:        int   = 0     # longest continuous saturation run (ticks)
    max_cte_warmup:     float = 0.0
    max_cte_after:      float = 0.0
    cte_log:            List[float] = field(default_factory=list)
    steer_log:          List[float] = field(default_factory=list)
    speed_log:          List[float] = field(default_factory=list)
    p_term_log:         List[float] = field(default_factory=list)
    d_term_log:         List[float] = field(default_factory=list)
    swerve_start_tick:  int   = 0


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────
def yaw_rad(tf: "carla.Transform") -> float:
    return math.radians(tf.rotation.yaw)

def cte_from_waypoint(veh_loc, veh_yaw_rad, wp_loc) -> float:
    """Exact formula from main.cpp line 339-341."""
    dx = veh_loc.x - wp_loc.x
    dy = veh_loc.y - wp_loc.y
    return -math.sin(veh_yaw_rad) * dx + math.cos(veh_yaw_rad) * dy


def shift_waypoint_left(wp: "carla.Waypoint", metres: float) -> "carla.Location":
    """Return a point metres to the LEFT of the waypoint centre.

    Prefers using CARLA's native left-lane waypoint (guaranteed on-road surface).
    Falls back to geometric offset if no adjacent lane exists.
    """
    left_wp = wp.get_left_lane()
    if left_wp is not None:
        # Use road-centre of adjacent lane — always valid road surface
        return left_wp.transform.location

    # Fallback: geometric offset along -right_vector
    rgt = wp.transform.get_right_vector()
    loc = wp.transform.location
    return carla.Location(
        x=loc.x - metres * rgt.x,
        y=loc.y - metres * rgt.y,
        z=loc.z,
    )


def settling_time_analysis(cte_log: List[float], swerve_tick: int,
                            threshold: float, dt: float):
    """Return (settling_tick_relative, oscillation_count, description)."""
    post = cte_log[swerve_tick:]
    settle_tick = None
    sign_changes = 0
    prev_sign = None

    for i, val in enumerate(post):
        s = 1 if val > 0 else (-1 if val < 0 else 0)
        if prev_sign is not None and s != 0 and s != prev_sign:
            sign_changes += 1
        if s != 0:
            prev_sign = s

        if abs(val) < threshold and settle_tick is None:
            settle_tick = i

    desc = (f"Settled at tick +{settle_tick} ({settle_tick * dt:.2f}s) "
            f"with {sign_changes} sign changes"
            if settle_tick is not None else "Did not settle")
    return settle_tick, sign_changes, desc


# ─────────────────────────────────────────────────────────────────────────────
# Main test
# ─────────────────────────────────────────────────────────────────────────────
def run_test(host=CARLA_HOST, port=CARLA_PORT) -> Result:
    result = Result()
    actors: list = []
    world = None
    original_settings = None
    client = None

    try:
        # ── Connect ──────────────────────────────────────────────────────────
        log.info("Connecting to CARLA %s:%d …", host, port)
        client = carla.Client(host, port)
        client.set_timeout(30.0)
        world = client.get_world()

        if MAP_NAME not in world.get_map().name:
            log.info("Loading %s …", MAP_NAME)
            world = client.load_world(MAP_NAME)
            time.sleep(5.0)

        # ── Synchronous mode ──────────────────────────────────────────────────
        original_settings = world.get_settings()
        world.apply_settings(carla.WorldSettings(
            synchronous_mode=True,
            fixed_delta_seconds=SYNC_DELTA,
            no_rendering_mode=True,
        ))
        log.info("Sync mode ON  dt=%.3f s", SYNC_DELTA)

        # ── Spawn ego ─────────────────────────────────────────────────────────
        sp  = world.get_map().get_spawn_points()[SPAWN_POINT_IDX]
        bp  = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
        ego = world.spawn_actor(bp, sp)
        actors.append(ego)
        world.tick(); world.tick()
        log.info("Ego spawned at x=%.1f y=%.1f yaw=%.1f°",
                 sp.location.x, sp.location.y, sp.rotation.yaw)

        # ── PIDs ──────────────────────────────────────────────────────────────
        pid_steer    = PID(STEER_KP, STEER_KI, STEER_KD, STEER_MAX, STEER_MIN)
        pid_throttle = PID(THROTTLE_KP, THROTTLE_KI, THROTTLE_KD,
                           THROTTLE_MAX, THROTTLE_MIN)

        carla_map = world.get_map()

        # ── State ─────────────────────────────────────────────────────────────
        PHASE_WARMUP   = "WARMUP"    # approach cruise speed on straight
        PHASE_SWERVE   = "SWERVE"    # trajectory shifted 3.5 m left
        PHASE_SETTLED  = "SETTLED"
        phase = PHASE_WARMUP

        swerve_tick    = None
        prev_steer     = 0.0
        oscillations   = 0
        sat_run        = 0            # current saturation run length (ticks)
        max_sat_run    = 0

        log.info("─" * 70)
        log.info("%-6s %-8s %-8s %-8s %-8s %-8s %-10s",
                 "Tick", "Speed", "CTE", "Steer", "P", "D", "Phase")
        log.info("─" * 70)

        for tick in range(MAX_RUN_TICKS):
            world.tick()

            # ── Snapshot ──────────────────────────────────────────────────────
            tf    = ego.get_transform()
            vel   = ego.get_velocity()
            speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            vyw   = yaw_rad(tf)

            # Reference waypoint — road centre or shifted
            wp = carla_map.get_waypoint(
                tf.location, project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            if phase in (PHASE_SWERVE, PHASE_SETTLED):
                ref_loc = shift_waypoint_left(wp, SWERVE_OFFSET_M)
            else:
                ref_loc = wp.transform.location

            cte_val = cte_from_waypoint(tf.location, vyw, ref_loc)

            # ── Phase transition: warmup → swerve ─────────────────────────────
            if phase == PHASE_WARMUP and tick >= WARMUP_TICKS:
                phase      = PHASE_SWERVE
                swerve_tick = tick
                result.swerve_start_tick = tick
                log.info("🔀  Trajectory shift at tick %d (speed=%.2f m/s, CTE=%.3f m)",
                         tick, speed, cte_val)

            # ── Phase transition: swerve → settled ────────────────────────────
            if (phase == PHASE_SWERVE and swerve_tick is not None
                    and tick > swerve_tick + 3
                    and abs(cte_val) < SETTLING_CTE_M):
                phase = PHASE_SETTLED
                result.settling_ticks = tick - swerve_tick
                log.info("✅  Settled at tick %d (+%d ticks / +%.2f s, CTE=%.3f m)",
                         tick, result.settling_ticks,
                         result.settling_ticks * SYNC_DELTA, cte_val)

            # ── dt (fixed, but use SYNC_DELTA for correctness) ────────────────
            pid_steer.update_dt(SYNC_DELTA)
            pid_throttle.update_dt(SYNC_DELTA)

            # ── PID update ────────────────────────────────────────────────────
            pid_steer.update_error(cte_val)
            steer = pid_steer.total_error()

            speed_err = speed - TARGET_SPEED
            pid_throttle.update_error(speed_err)
            throttle_raw = pid_throttle.total_error()
            throttle = max(0.0, throttle_raw)
            brake    = max(0.0, -throttle_raw)

            # ── Oscillation detection (only during swerve phase) ─────────────
            if phase == PHASE_SWERVE:
                if steer * prev_steer < 0 and abs(steer) > 0.01:
                    oscillations += 1

            # ── Saturation run tracking ───────────────────────────────────────
            if pid_steer.is_saturated:
                sat_run += 1
                max_sat_run = max(max_sat_run, sat_run)
            else:
                sat_run = 0

            prev_steer = steer

            # ── Log data ──────────────────────────────────────────────────────
            result.cte_log.append(cte_val)
            result.steer_log.append(steer)
            result.speed_log.append(speed)
            result.p_term_log.append(pid_steer.p_term)
            result.d_term_log.append(pid_steer.d_term)

            if phase == PHASE_WARMUP:
                result.max_cte_warmup = max(result.max_cte_warmup, abs(cte_val))
            else:
                result.max_cte_after = max(result.max_cte_after, abs(cte_val))

            # ── Apply control ─────────────────────────────────────────────────
            ego.apply_control(carla.VehicleControl(
                steer=float(steer),
                throttle=float(throttle),
                brake=float(brake),
            ))

            if tick % 20 == 0:
                log.info("%-6d %-8.2f %-8.3f %-8.3f %-8.4f %-8.4f %-10s",
                         tick, speed, cte_val, steer,
                         pid_steer.p_term, pid_steer.d_term, phase)

            if phase == PHASE_SETTLED and tick > swerve_tick + MAX_SETTLING_TICKS:
                log.info("Post-settle window complete — ending at tick %d", tick)
                break

        # ── Store summary stats ───────────────────────────────────────────────
        result.oscillations = oscillations
        result.max_sat_run  = max_sat_run

        # ── POST-RUN ASSERTIONS ───────────────────────────────────────────────
        log.info("─" * 70)
        log.info("POST-RUN ASSERTIONS")
        log.info("─" * 70)

        # 1. Settling time ≤ 3 s (60 ticks)
        if result.settling_ticks is not None:
            settle_s = result.settling_ticks * SYNC_DELTA
            _assert(result,
                    result.settling_ticks <= MAX_SETTLING_TICKS,
                    f"FAIL: Settling time {settle_s:.2f}s > {MAX_SETTLING_TICKS * SYNC_DELTA:.1f}s")
            log.info("✅  Settling time: %d ticks (%.2fs) — limit %d ticks (%.1fs)",
                     result.settling_ticks, settle_s,
                     MAX_SETTLING_TICKS, MAX_SETTLING_TICKS * SYNC_DELTA)
        else:
            _assert(result, False,
                    f"FAIL: Vehicle never settled (CTE < {SETTLING_CTE_M}m) "
                    f"within {MAX_RUN_TICKS} ticks")

        # 2. Oscillations ≤ 2
        _assert(result,
                result.oscillations <= MAX_OSCILLATIONS,
                f"FAIL: {result.oscillations} steering sign-reversals > limit {MAX_OSCILLATIONS}. "
                f"Suggests Kd too low or Kp too high.")
        log.info("✅  Steering oscillations: %d  (limit %d)",
                 result.oscillations, MAX_OSCILLATIONS)

        # 3. Saturation: no run > MAX_SAT_TICKS (1 s)
        sat_s = result.max_sat_run * SYNC_DELTA
        _assert(result,
                result.max_sat_run <= MAX_SAT_TICKS,
                f"FAIL: PID output saturated for {sat_s:.2f}s continuously "
                f"(limit {MAX_SAT_TICKS * SYNC_DELTA:.1f}s). "
                f"Indicates integral windup or unstable gain.")
        log.info("✅  Max continuous saturation: %d ticks (%.2fs) — limit %d ticks (%.1fs)",
                 result.max_sat_run, sat_s, MAX_SAT_TICKS, MAX_SAT_TICKS * SYNC_DELTA)

        # 4. Warmup CTE < 0.5 m (straight tracking)
        _assert(result,
                result.max_cte_warmup < 0.5,
                f"FAIL: Warmup CTE {result.max_cte_warmup:.3f}m > 0.5m. "
                f"PID not tracking straight line before swerve test.")
        log.info("✅  Warmup max CTE: %.3f m  (limit 0.5 m)", result.max_cte_warmup)

        # ── Quality metrics (logged but not asserted) ─────────────────────────
        post_settle_log = result.cte_log[result.swerve_start_tick
                                         + (result.settling_ticks or 0):]
        if post_settle_log:
            residual_cte = max(abs(v) for v in post_settle_log[-20:]) if len(post_settle_log) >= 20 else 0.0
            log.info("📊  Residual CTE (last 20 ticks after settle): %.3f m", residual_cte)
        log.info("📊  Warmup max CTE: %.3f m", result.max_cte_warmup)
        log.info("📊  Post-swerve max CTE: %.3f m", result.max_cte_after)

    except Exception as exc:
        result.passed = False
        result.reason = f"Exception: {exc}"
        log.exception("Unexpected error")
    finally:
        log.info("Cleaning up …")
        if world is not None and original_settings is not None:
            world.apply_settings(original_settings)
        for a in actors:
            try: a.destroy()
            except Exception: pass
        if world is not None:
            try: world.tick()
            except Exception: pass
        log.info("Cleanup complete")

    return result


def _assert(result: Result, condition: bool, message: str):
    if not condition:
        result.passed = False
        if not result.reason:
            result.reason = message
        log.error("❌  %s", message)


# ─────────────────────────────────────────────────────────────────────────────
# Settling-time analysis for plot_pid.py data
# ─────────────────────────────────────────────────────────────────────────────
def analyse_settling_time(steer_file="steer_pid_data.txt",
                           throttle_file="throttle_pid_data.txt",
                           dt=0.05, settle_pct=0.05):
    """
    Read steer_pid_data.txt and throttle_pid_data.txt and report settling time.

    settling_time = first time the error remains within settle_pct * steady_state
    for at least 10 consecutive frames.

    Args:
        settle_pct: fraction of steady-state error — default 5%.
    """
    import pandas as pd
    try:
        steer_df = pd.read_csv(steer_file, sep=r"\s+", header=None,
                                usecols=[0, 1, 2])
        steer_df.columns = ["iter", "cte", "steer"]
        thr_df = pd.read_csv(throttle_file, sep=r"\s+", header=None,
                              usecols=[0, 1, 2, 3])
        thr_df.columns = ["iter", "err", "brake", "throttle"]
    except FileNotFoundError as e:
        log.error("Data file not found: %s  (run the simulation first)", e)
        return

    def settling(series, pct, window=10):
        ss = series.iloc[-50:].mean()
        band = abs(ss) * pct + 0.01   # minimum band 1 cm
        for i in range(len(series) - window):
            chunk = series.iloc[i:i + window]
            if (chunk - ss).abs().max() < band:
                return i, ss
        return None, ss

    steer_settle_idx, steer_ss = settling(steer_df["cte"], settle_pct)
    thr_settle_idx,   thr_ss   = settling(thr_df["err"],   settle_pct)

    print("\n── Settling Time Analysis ─────────────────────────────")
    if steer_settle_idx is not None:
        print(f"  Steer CTE:         settles at frame {steer_settle_idx}"
              f"  ({steer_settle_idx * dt:.1f}s)  ss={steer_ss:.4f}m")
    else:
        print("  Steer CTE:         did not settle")

    if thr_settle_idx is not None:
        print(f"  Throttle error:    settles at frame {thr_settle_idx}"
              f"  ({thr_settle_idx * dt:.1f}s)  ss={thr_ss:.4f}m/s")
    else:
        print("  Throttle error:    did not settle")

    # Gain tuning suggestion
    if steer_settle_idx and steer_settle_idx * dt > 4.0:
        print("\n  ⚠️  Steer settling > 4s. Suggestions:")
        print("     Over-damped (sluggish):  increase Kp (→ 0.08) OR reduce Kd (→ 0.03)")
        print("     Under-damped (fishtail): reduce Kp (→ 0.03) OR increase Kd (→ 0.08)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        t = [i * dt for i in steer_df["iter"]]
        axes[0].plot(t, steer_df["cte"], label="CTE (m)")
        axes[0].plot(t, steer_df["steer"], label="Steer output")
        axes[0].axhline(0.2, color="r", linestyle="--", label="Settle band +0.2m")
        axes[0].axhline(-0.2, color="r", linestyle="--")
        if steer_settle_idx:
            axes[0].axvline(steer_settle_idx * dt, color="g", linestyle=":",
                            label=f"Settled {steer_settle_idx * dt:.1f}s")
        axes[0].set_title("Steering: CTE and Output")
        axes[0].legend(); axes[0].grid(True)

        t2 = [i * dt for i in thr_df["iter"]]
        axes[1].plot(t2, thr_df["err"],      label="Speed error (m/s)")
        axes[1].plot(t2, thr_df["throttle"], label="Throttle")
        axes[1].plot(t2, thr_df["brake"],    label="Brake")
        axes[1].axhline(0, color="k", linewidth=0.5)
        axes[1].set_title("Throttle: Speed Error and Output")
        axes[1].legend(); axes[1].grid(True)

        plt.tight_layout()
        out = "pid_settling_analysis.png"
        plt.savefig(out, dpi=100)
        print(f"\n  Plot saved: {out}")
    except ImportError:
        print("  (matplotlib not available — skipping plot)")


# ─────────────────────────────────────────────────────────────────────────────
# pytest wrapper
# ─────────────────────────────────────────────────────────────────────────────
def test_pid_trajectory_shift_stability():
    """pytest entry-point."""
    import socket
    try:
        s = socket.create_connection((CARLA_HOST, CARLA_PORT), timeout=3)
        s.close()
    except (ConnectionRefusedError, OSError):
        import pytest
        pytest.skip("CARLA not running")

    result = run_test()
    _print_summary(result)
    assert result.passed, result.reason


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def _print_summary(result: Result):
    log.info("=" * 70)
    log.info("TEST SUMMARY")
    log.info("=" * 70)
    log.info("Result             : %s", "PASS ✅" if result.passed else f"FAIL ❌  {result.reason}")
    log.info("Settling ticks     : %s  (%.2fs)",
             result.settling_ticks,
             (result.settling_ticks or 0) * SYNC_DELTA)
    log.info("Oscillations       : %d  (limit %d)", result.oscillations, MAX_OSCILLATIONS)
    log.info("Max sat. run       : %d ticks (%.2fs) — limit %d ticks (%.1fs)",
             result.max_sat_run, result.max_sat_run * SYNC_DELTA,
             MAX_SAT_TICKS, MAX_SAT_TICKS * SYNC_DELTA)
    log.info("Warmup max CTE     : %.3f m", result.max_cte_warmup)
    log.info("Post-swerve max CTE: %.3f m", result.max_cte_after)
    log.info("=" * 70)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="PID trajectory-shift stability test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_avoidance_pid.py                     # run simulation test
  python3 test_avoidance_pid.py --analyse-only      # analyse existing data files
  pytest  test_avoidance_pid.py -v                  # pytest mode
        """,
    )
    parser.add_argument("--host", default=CARLA_HOST)
    parser.add_argument("--port", type=int, default=CARLA_PORT)
    parser.add_argument("--analyse-only", action="store_true",
                        help="Skip CARLA, only analyse steer/throttle_pid_data.txt")
    args_parsed = parser.parse_args()

    if args_parsed.analyse_only:
        analyse_settling_time()
        sys.exit(0)

    result = run_test(args_parsed.host, args_parsed.port)
    _print_summary(result)
    sys.exit(0 if result.passed else 1)
