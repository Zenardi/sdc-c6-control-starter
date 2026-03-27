"""
test_lane_change_pid.py
=======================
AV QA Engineer — Obstacle-Triggered EMERGENCY Lane-Change PID Stability Test

Scene (Town03):
  Ego spawned near (x=-10, y=140, z=2), snapped to road centre via CARLA
  waypoint API.
  Stationary police car (vehicle.dodge.charger_police) placed exactly 20 m
  ahead in the SAME lane using waypoint.next(20.0) — guaranteed centred and
  DIRECTLY in the ego's path.

  Why 20 m?  At ~4.6 m/s the car would hit the police car in ≈4.3 s without
  intervention.  This is a genuine collision scenario, not a planned merge.

  At 15 m distance the lane-change fires (≈3.3 s before impact at cruise
  speed).  The PID must swerve the car into the adjacent left lane in time.

  Expected near-miss: when the ego's x-position equals the police car's
  x-position (~3.3 s post-trigger), the car has moved ≈3.1 m laterally —
  a centre-to-centre clearance of ≈3.1 m.  Safe but close.

PID gains (exact mirror of main.cpp, never diverge):
  Steering  : Kp=0.05  Ki=0.00  Kd=0.05  clamp=[−0.3, +0.3]
  Throttle  : Kp=0.10  Ki=0.001 Kd=0.05  clamp=[−1.0, +1.0]
  CTE formula : cte = −sin(yaw)·Δx + cos(yaw)·Δy   (from main.cpp line 341)

Architecture note
-----------------
Standalone Python test — PID math mirrors pid_controller.cpp so the test
runs without the C++ WebSocket binary.  Running run_main_pid.sh alongside a
second CARLA Python client is unsafe (two clients fight for the same actor
and WebSocket port 4567).

Pass / Fail criteria:
  1. Ego reaches new lane centreline (CTE < 0.2 m) within 5 s (100 ticks).
  2. Max overshoot beyond new centre ≤ 0.5 m.
  3. Zero collision (min distance to police car ≥ 0.5 m at all times).
  4. Steering oscillations ≤ 3 sign-reversals during the manoeuvre.
  5. [Near-miss] min distance to police car < 5.0 m (proves genuine avoidance,
     not a trivially wide detour).

Usage:
    # CARLA must be running (headless or with display):
    #   cd project && ./CARLA/CarlaUE4.sh -RenderOffScreen -carla-world-port=2000 &
    cd project
    python3 test_lane_change_pid.py          # standalone
    pytest  test_lane_change_pid.py -v       # pytest
"""

import sys
import math
import time
import logging
from dataclasses import dataclass
from typing import Optional

logging.basicConfig(level=logging.INFO, format="%(levelname)-5s %(message)s")
log = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# CARLA import guard
# ─────────────────────────────────────────────────────────────────────────────
try:
    import carla
except ImportError:
    log.error("carla module not found — add CARLA Python API to PYTHONPATH")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
CARLA_HOST     = "localhost"
CARLA_PORT     = 2000
SYNC_DELTA     = 0.05              # s — deterministic physics tick

MAP_NAME       = "Town03"
# User-specified spawn; snapped to road centre automatically.
EGO_SPAWN_LOC  = carla.Location(x=-10.0, y=140.0, z=2.0)

POLICE_DIST_M  = 20.0             # m — police car ahead: close enough to be on collision course
TRIGGER_DIST_M = 15.0             # m — emergency trigger (~3.3 s before impact at cruise speed)
LOOKAHEAD_M    = 10.0             # m — waypoint look-ahead for CTE

TARGET_SPEED   = 30.0 / 3.6      # 8.33 m/s = 30 km/h

# PID gains — exact mirror of main.cpp (never change independently)
STEER_KP,    STEER_KI,    STEER_KD    = 0.05,  0.0,   0.05
STEER_MAX,   STEER_MIN                = 0.3,  -0.3
THROTTLE_KP, THROTTLE_KI, THROTTLE_KD = 0.10,  0.001, 0.05
THROTTLE_MAX, THROTTLE_MIN            = 1.0,  -1.0

# Test parameters
MAX_SETTLING_TICKS = 100         # 5 s at dt=0.05 s (tighter trigger needs more room)
MAX_OVERSHOOT_M    = 0.5         # m beyond new lane centre
MAX_OSCILLATIONS   = 3           # steer sign-reversals during manoeuvre
COLLISION_DIST_M   = 0.5         # m — below this = collision
NEAR_MISS_MAX_M    = 5.0         # m — min dist MUST be below this (proves genuine avoidance)
MAX_RUN_TICKS      = 600         # hard safety ceiling


# ─────────────────────────────────────────────────────────────────────────────
# SimplePID — mirrors pid_controller.cpp byte-for-byte
# ─────────────────────────────────────────────────────────────────────────────
class SimplePID:
    def __init__(self, kp: float, ki: float, kd: float,
                 out_max: float, out_min: float) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.out_min = out_max, out_min
        self.p_err = self.i_err = self.d_err = 0.0
        self.prev_cte = 0.0

    def update(self, cte: float) -> None:
        self.d_err  = (cte - self.prev_cte) / SYNC_DELTA
        self.p_err  = cte
        self.i_err += cte * SYNC_DELTA
        self.prev_cte = cte

    def output(self) -> float:
        raw = self.kp * self.p_err + self.ki * self.i_err + self.kd * self.d_err
        return max(self.out_min, min(self.out_max, raw))


# ─────────────────────────────────────────────────────────────────────────────
# Result dataclass
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Result:
    pass_fail:         bool          = False
    reason:            str           = ""
    settling_ticks:    Optional[int] = None
    max_overshoot:     float         = 0.0
    min_dist_police:   float         = float("inf")
    min_dist_at_pass:  float         = float("inf")   # dist when ego x == police x
    oscillations:      int           = 0
    triggered:         bool          = False


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────
def dist2d(a: carla.Location, b: carla.Location) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def yaw_diff_deg(y1: float, y2: float) -> float:
    """Signed shortest-path angular difference, wrapped to (−180, 180]."""
    return ((y1 - y2) + 180.0) % 360.0 - 180.0


def get_lookahead_wp(carla_map, loc: carla.Location,
                     use_left: bool = False) -> "carla.Waypoint":
    """Return the waypoint LOOKAHEAD_M ahead in the current (or left) lane.

    If use_left=True: prefer the left-lane waypoint (always on valid road
    surface via CARLA road network API).  Falls back to current lane if no
    same-direction left lane exists.
    """
    wp = carla_map.get_waypoint(
        loc, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    if use_left:
        left = wp.get_left_lane()
        if (left is not None
                and left.lane_type == carla.LaneType.Driving
                and abs(yaw_diff_deg(left.transform.rotation.yaw,
                                     wp.transform.rotation.yaw)) < 90.0):
            wp = left
    nexts = wp.next(LOOKAHEAD_M)
    return nexts[0] if nexts else wp


def compute_cte(ego_loc: carla.Location, ego_yaw_deg: float,
                target_wp: "carla.Waypoint") -> float:
    """Signed lateral cross-track error (main.cpp line 341).

    Positive  = ego is to the RIGHT of the target  (must steer left / −steer).
    Negative  = ego is to the LEFT  of the target  (must steer right / +steer).
    """
    yaw = math.radians(ego_yaw_deg)
    dx  = target_wp.transform.location.x - ego_loc.x
    dy  = target_wp.transform.location.y - ego_loc.y
    return -math.sin(yaw) * dx + math.cos(yaw) * dy


# ─────────────────────────────────────────────────────────────────────────────
# Main test
# ─────────────────────────────────────────────────────────────────────────────
def test_lane_change_pid() -> Result:
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(30.0)
    log.info("Connecting to CARLA %s:%d …", CARLA_HOST, CARLA_PORT)

    world         = client.get_world()
    orig_settings = world.get_settings()
    actors        = []

    try:
        # ── Load map ─────────────────────────────────────────────────────────
        if MAP_NAME not in world.get_map().name:
            log.info("Loading %s …", MAP_NAME)
            world = client.load_world(MAP_NAME)
            time.sleep(5.0)
            world.tick()

        carla_map = world.get_map()

        # ── Enable synchronous mode ──────────────────────────────────────────
        settings = world.get_settings()
        settings.synchronous_mode   = True
        settings.fixed_delta_seconds = SYNC_DELTA
        world.apply_settings(settings)
        log.info("Sync mode ON  dt=%.3f s", SYNC_DELTA)

        # ── Snap EGO_SPAWN_LOC to road centre ────────────────────────────────
        ego_wp = carla_map.get_waypoint(
            EGO_SPAWN_LOC, project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        ego_tf = ego_wp.transform
        ego_tf.location.z += 0.5            # small lift to avoid ground clipping

        ego_bp = world.get_blueprint_library().find("vehicle.lincoln.mkz_2017")
        ego_bp.set_attribute("role_name", "ego")
        ego = world.try_spawn_actor(ego_bp, ego_tf)

        # Fallback: search spawn points within 30 m if road snap failed
        if ego is None:
            log.warning("Snap spawn failed — scanning nearby spawn points …")
            for sp in carla_map.get_spawn_points():
                if dist2d(sp.location, EGO_SPAWN_LOC) < 30.0:
                    ego = world.try_spawn_actor(ego_bp, sp)
                    if ego is not None:
                        log.info("Fallback spawn at x=%.1f y=%.1f",
                                 sp.location.x, sp.location.y)
                        ego_wp = carla_map.get_waypoint(
                            sp.location, project_to_road=True,
                            lane_type=carla.LaneType.Driving
                        )
                        break

        if ego is None:
            raise RuntimeError(
                "Could not spawn ego near (x=-10, y=140, z=2) in Town03. "
                "Adjust EGO_SPAWN_LOC to a valid road location."
            )

        actors.append(ego)
        world.tick()

        ego_loc = ego.get_transform().location
        log.info("Ego spawned at x=%.1f y=%.1f yaw=%.1f°",
                 ego_loc.x, ego_loc.y, ego.get_transform().rotation.yaw)

        # ── Verify a same-direction left lane exists ─────────────────────────
        left_wp = ego_wp.get_left_lane()
        if (left_wp is None
                or left_wp.lane_type != carla.LaneType.Driving
                or abs(yaw_diff_deg(left_wp.transform.rotation.yaw,
                                    ego_wp.transform.rotation.yaw)) >= 90.0):
            # Try 15 m ahead (road might narrow at spawn)
            nxt = ego_wp.next(15.0)
            if nxt:
                left_wp = nxt[0].get_left_lane()

        if (left_wp is None
                or left_wp.lane_type != carla.LaneType.Driving
                or abs(yaw_diff_deg(left_wp.transform.rotation.yaw,
                                    ego_wp.transform.rotation.yaw)) >= 90.0):
            raise RuntimeError(
                "SKIP: No same-direction adjacent driving lane at spawn. "
                "The chosen coordinate is on a single-lane or one-way road. "
                "Try a different EGO_SPAWN_LOC on a multi-lane section."
            )

        log.info("Left lane confirmed — road_id=%d lane_id=%d  yaw=%.1f°",
                 left_wp.road_id, left_wp.lane_id,
                 left_wp.transform.rotation.yaw)

        # ── Police car — 50 m ahead in the SAME lane ────────────────────────
        nexts = ego_wp.next(POLICE_DIST_M)
        if not nexts:
            raise RuntimeError("No waypoint found 50 m ahead of ego spawn.")
        police_wp = nexts[0]
        police_tf = police_wp.transform
        police_tf.location.z += 0.1        # slight lift to avoid floor clipping

        police_bp = world.get_blueprint_library().find(
            "vehicle.dodge.charger_police"
        )
        police_bp.set_attribute("role_name", "police")
        police = world.try_spawn_actor(police_bp, police_tf)
        if police is None:
            raise RuntimeError("Could not spawn police car 50 m ahead.")

        # Lock it in place
        police.apply_control(carla.VehicleControl(brake=1.0))
        police.set_target_velocity(carla.Vector3D(0, 0, 0))
        actors.append(police)
        world.tick()

        actual_police_loc = police.get_transform().location
        log.info("Police car at x=%.1f y=%.1f (actual dist=%.1f m)",
                 actual_police_loc.x, actual_police_loc.y,
                 dist2d(ego_loc, actual_police_loc))

        # ── PID controllers ──────────────────────────────────────────────────
        steer_pid    = SimplePID(STEER_KP,    STEER_KI,    STEER_KD,
                                 STEER_MAX,   STEER_MIN)
        throttle_pid = SimplePID(THROTTLE_KP, THROTTLE_KI, THROTTLE_KD,
                                 THROTTLE_MAX, THROTTLE_MIN)

        # ── Test state ───────────────────────────────────────────────────────
        phase         = "APPROACH"   # APPROACH → SWERVE → SETTLED
        triggered     = False
        swerve_tick   = 0
        settling_tick: Optional[int] = None
        max_overshoot = 0.0
        min_dist      = float("inf")
        min_dist_at_pass = float("inf")  # min dist recorded AFTER trigger
        oscillations  = 0
        prev_steer    = 0.0
        sat_run       = 0
        max_sat_run   = 0

        result = Result()

        # ── Header ───────────────────────────────────────────────────────────
        log.info("─" * 84)
        log.info("%-6s %-7s %-8s %-8s %-9s %-9s %-8s",
                 "Tick", "Speed", "CTE", "Steer",
                 "DistPol", "Overshoot", "Phase")
        log.info("─" * 84)

        # ── Main loop ────────────────────────────────────────────────────────
        for tick in range(MAX_RUN_TICKS):
            world.tick()

            t         = ego.get_transform()
            ego_loc   = t.location
            ego_yaw   = t.rotation.yaw
            vel       = ego.get_velocity()
            speed     = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

            dist_pol  = dist2d(ego_loc, police.get_transform().location)
            if dist_pol < min_dist:
                min_dist = dist_pol
                # Minimum distance == the lateral clearance when car is directly
                # alongside the police car (dist stops decreasing at that moment).
                # Track it as min_dist_at_pass — no coordinate math needed.
                if triggered:
                    min_dist_at_pass = min_dist

            # ── Collision guard ──────────────────────────────────────────────
            if dist_pol < COLLISION_DIST_M:
                result.reason = (f"COLLISION at tick {tick}: "
                                 f"distance={dist_pol:.2f} m < {COLLISION_DIST_M} m")
                log.error("❌  %s", result.reason)
                break

            # ── Trigger: switch to left lane when within 25 m ────────────────
            if not triggered and dist_pol < TRIGGER_DIST_M:
                triggered   = True
                phase       = "SWERVE"
                swerve_tick = tick
                log.info("🔀  Lane-change triggered at tick %d "
                         "(speed=%.2f m/s, dist_police=%.1f m)",
                         tick, speed, dist_pol)

            # ── Trajectory: right lane (approach) or left lane (swerve) ──────
            target_wp = get_lookahead_wp(carla_map, ego_loc, use_left=triggered)
            cte       = compute_cte(ego_loc, ego_yaw, target_wp)

            # ── PID compute ──────────────────────────────────────────────────
            speed_err = TARGET_SPEED - speed
            steer_pid.update(cte)
            throttle_pid.update(speed_err)

            steer    = steer_pid.output()
            raw_thr  = throttle_pid.output()
            throttle = max(0.0,  raw_thr)
            brake    = max(0.0, -raw_thr)

            ego.apply_control(carla.VehicleControl(
                throttle=float(throttle),
                steer=float(steer),
                brake=float(brake),
                manual_gear_shift=False,
            ))

            # ── Post-trigger metrics ─────────────────────────────────────────
            if triggered:
                # Oscillation: count steer sign-reversals (skip first 2 ticks
                # of initial transient)
                if tick > swerve_tick + 2 and prev_steer * steer < 0:
                    oscillations += 1

                prev_steer = steer

                # Saturation run length
                if abs(steer) >= STEER_MAX - 1e-4:
                    sat_run     += 1
                    max_sat_run  = max(max_sat_run, sat_run)
                else:
                    sat_run = 0

                # Settling detection
                if settling_tick is None and abs(cte) < 0.2:
                    settling_tick = tick
                    phase         = "SETTLED"
                    log.info("✅  Settled at tick %d (+%d ticks / +%.2f s,"
                             " CTE=%.3f m)",
                             tick, tick - swerve_tick,
                             (tick - swerve_tick) * SYNC_DELTA, cte)

                # Overshoot: CTE positive means ego overshot to the right of
                # the new (left) lane centre; record largest excursion after
                # first settling
                if settling_tick is not None and cte > 0.0:
                    if cte > max_overshoot:
                        max_overshoot = cte

                # End run 20 ticks after settling (stability window)
                if settling_tick is not None and tick >= settling_tick + 20:
                    break

            # ── Periodic log ─────────────────────────────────────────────────
            log_this = (tick % 20 == 0
                        or (triggered and tick < swerve_tick + 120 and tick % 5 == 0))
            if log_this:
                log.info("%-6d %-7.2f %-8.3f %-8.3f %-9.1f %-9.3f %-8s",
                         tick, speed, cte, steer, dist_pol,
                         max_overshoot, phase)

        # ─────────────────────────────────────────────────────────────────────
        # Assertions
        # ─────────────────────────────────────────────────────────────────────
        log.info("─" * 84)
        log.info("POST-RUN ASSERTIONS")
        log.info("─" * 84)

        failures = []

        if not triggered:
            failures.append(
                "TRIGGER: ego never came within 15 m of police car "
                f"(min dist observed={min_dist:.1f} m)"
            )

        if result.reason:           # collision mid-loop
            failures.append(result.reason)

        if settling_tick is None:
            failures.append(
                f"SETTLING: vehicle never settled (CTE < 0.2 m) "
                f"within {MAX_RUN_TICKS} ticks"
            )
        else:
            elapsed = settling_tick - swerve_tick
            label   = f"{elapsed} ticks ({elapsed * SYNC_DELTA:.2f} s)"
            limit   = f"{MAX_SETTLING_TICKS} ticks ({MAX_SETTLING_TICKS * SYNC_DELTA:.2f} s)"
            if elapsed > MAX_SETTLING_TICKS:
                failures.append(f"SETTLING: {label} > limit {limit}")
            else:
                log.info("✅  Settling : %s  (limit %s)", label, limit)

        if max_overshoot > MAX_OVERSHOOT_M:
            failures.append(
                f"OVERSHOOT: {max_overshoot:.3f} m > limit {MAX_OVERSHOOT_M} m"
            )
        else:
            log.info("✅  Overshoot: %.3f m  (limit %.1f m)",
                     max_overshoot, MAX_OVERSHOOT_M)

        if min_dist < COLLISION_DIST_M and not any("COLLISION" in f for f in failures):
            failures.append(f"COLLISION: min dist {min_dist:.2f} m < {COLLISION_DIST_M} m")
        else:
            log.info("✅  No collision  (min dist=%.2f m)", min_dist)

        # Near-miss assertion: car must have gotten genuinely close (proves avoidance, not wide detour)
        if min_dist_at_pass == float("inf"):
            failures.append("NEAR-MISS: lane-change never triggered or car never approached police")
        elif min_dist_at_pass > NEAR_MISS_MAX_M:
            failures.append(
                f"NEAR-MISS: closest approach post-trigger={min_dist_at_pass:.2f} m > {NEAR_MISS_MAX_M} m "
                f"(police car was NOT genuinely blocking — spawn distance too large or wrong lane)"
            )
        else:
            log.info("✅  Near-miss OK: %.2f m min clearance  (genuine avoidance, limit < %.1f m)",
                     min_dist_at_pass, NEAR_MISS_MAX_M)

        if oscillations > MAX_OSCILLATIONS:
            failures.append(
                f"OSCILLATIONS: {oscillations} > limit {MAX_OSCILLATIONS}"
            )
        else:
            log.info("✅  Oscillations: %d  (limit %d)",
                     oscillations, MAX_OSCILLATIONS)

        result.settling_ticks  = settling_tick
        result.max_overshoot   = max_overshoot
        result.min_dist_police = min_dist
        result.min_dist_at_pass = min_dist_at_pass
        result.oscillations    = oscillations
        result.triggered       = triggered

        if failures:
            result.pass_fail = False
            result.reason    = " | ".join(failures)
        elif not result.reason:
            result.pass_fail = True
            result.reason    = "All criteria met"

    finally:
        log.info("Cleaning up …")
        for a in actors:
            try:
                if a.is_alive:
                    a.destroy()
            except Exception:
                pass
        world.apply_settings(orig_settings)
        log.info("Sync mode restored — cleanup complete")

    # ── Summary ──────────────────────────────────────────────────────────────
    log.info("=" * 70)
    log.info("TEST SUMMARY")
    log.info("=" * 70)
    status = "PASS ✅" if result.pass_fail else "FAIL ❌"
    log.info("Result          : %s  %s", status, result.reason)
    log.info("Settling ticks  : %s  (%.2f s)",
             result.settling_ticks,
             (result.settling_ticks or 0) * SYNC_DELTA)
    log.info("Oscillations    : %d  (limit %d)",
             result.oscillations, MAX_OSCILLATIONS)
    log.info("Max overshoot   : %.3f m  (limit %.1f m)",
             result.max_overshoot, MAX_OVERSHOOT_M)
    log.info("Min dist police : %.2f m  (collision < %.1f m)",
             result.min_dist_police, COLLISION_DIST_M)
    log.info("Clearance@pass  : %.2f m  (genuine avoidance < %.1f m)",
             result.min_dist_at_pass, NEAR_MISS_MAX_M)
    log.info("=" * 70)

    return result


# ─────────────────────────────────────────────────────────────────────────────
# pytest entry-point
# ─────────────────────────────────────────────────────────────────────────────
def test_lane_change_pid_pytest():
    """pytest wrapper — SKIP if CARLA is not reachable."""
    try:
        import carla as _c
        _c.Client(CARLA_HOST, CARLA_PORT).set_timeout(3.0).get_world()
    except Exception:
        import pytest
        pytest.skip("CARLA not reachable — start the simulator first")

    result = test_lane_change_pid()
    assert result.pass_fail, result.reason


if __name__ == "__main__":
    r = test_lane_change_pid()
    sys.exit(0 if r.pass_fail else 1)
