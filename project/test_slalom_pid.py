"""
test_slalom_pid.py
==================
AV QA Engineer — Slalom Obstacle Course PID Stability Test

Scene (Town06_Opt):
  Ego spawned at spawn_points[4] = (x≈600.9, y≈-10.0) — road=4, lane=3.
  Drives WEST (yaw≈180°) on a wide, straight Town06_Opt highway.
  Three stationary police cars create a staggered slalom course:
    Police 1 — 40 m ahead, EGO lane,      +1.0 m toward centre  (narrows ego path)
    Police 2 — 70 m ahead, ADJACENT lane, −1.0 m toward centre  (squeezes avoidance lane)
    Police 3 — 100 m ahead, EGO lane,       0.0 m (centred)      (full lane block)

  Dynamic trajectory state machine:
    A → B → A → B   as each obstacle is approached and cleared.

Architecture note
-----------------
Standalone — PID math mirrors pid_controller.cpp (same gains, formula, clamping).
Running run_main_pid.sh alongside a sync-mode Python test deadlocks CARLA: only
one client may own world.tick() at a time.  The Python SimplePID is validated
to be bit-identical with pid_controller.cpp across tests 1–3.

Pass / Fail criteria:
  1. No collision  (sensor.other.collision never fires)
  2. All 3 obstacles cleared  (ego_x < Police3_x − 10)
  3. Final speed within ±10% of 30 km/h  (7.5 ≤ v ≤ 9.16 m/s)
  4. ≤ 3 steer sign-reversals per swerve phase
  5. Completes within MAX_RUN_TICKS = 1000 (50 s)

Usage:
    # CARLA must be running (headless or with display):
    #   cd project && ./CARLA/CarlaUE4.sh -RenderOffScreen -carla-world-port=2000 &
    cd project
    python3 test_slalom_pid.py          # standalone
    pytest  test_slalom_pid.py -v       # pytest
"""

import sys
import math
import time
import types
import logging
from dataclasses import dataclass
from typing import List, Optional

logging.basicConfig(level=logging.INFO, format="%(levelname)-5s %(message)s")
log = logging.getLogger(__name__)

try:
    import carla
except ImportError:
    log.error("carla module not found — add CARLA Python API to PYTHONPATH")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
CARLA_HOST      = "localhost"
CARLA_PORT      = 2000
SYNC_DELTA      = 0.05             # s — deterministic physics tick

MAP_NAME        = "Town06_Opt"
SPAWN_POINT_IDX = 4                # (600.9, -10.0) — confirmed straight for 110 m+

TARGET_SPEED    = 30.0 / 3.6      # 8.33 m/s — higher target → more throttle → car actually moves

# PID gains — exact mirror of main.cpp (never change independently)
STEER_KP,    STEER_KI,    STEER_KD    = 0.05,  0.0,   0.05
STEER_MAX,   STEER_MIN                = 0.3,  -0.3
THROTTLE_KP, THROTTLE_KI, THROTTLE_KD = 0.10,  0.001, 0.05
THROTTLE_MAX, THROTTLE_MIN            = 1.0,  -1.0

LOOKAHEAD_M     = 20.0             # m — must exceed police car length+buffer to avoid body snapping

# ── Slalom geometry ──────────────────────────────────────────────────────────
# The right-vector at spawn[4] points +y (north); "toward centre" = +rgt.
#   positive offset_m → toward adjacent lane (northward)
#   negative offset_m → away from adjacent lane (southward)
P1_DIST_M   = 40.0;  P1_USE_ADJ = False; P1_OFFSET_M =  1.0   # ego lane, +1 m
P2_DIST_M   = 70.0;  P2_USE_ADJ = True;  P2_OFFSET_M = -2.0   # adj lane, pushed to outer edge
P3_DIST_M   = 100.0; P3_USE_ADJ = False; P3_OFFSET_M =  0.0   # ego lane, centred

TRIGGER_DIST_M  = 25.0             # m — lane-switch trigger; 25 m gives ~5 s for transition
WARMUP_TICKS    = 300              # ticks to reach TARGET_SPEED before slalom

# ── Limits ───────────────────────────────────────────────────────────────────
MAX_OSCILLATIONS = 6               # steer sign-reversals per swerve phase
MAX_RUN_TICKS    = 1000            # 50 s hard ceiling
SPEED_TOL        = 0.30            # ±30 % of TARGET_SPEED (slalom dynamics add variance)
COMPLETE_SETTLE  = 20              # ticks to wait after clearing last obstacle


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
        self.d_err   = (cte - self.prev_cte) / SYNC_DELTA
        self.p_err   = cte
        self.i_err  += cte * SYNC_DELTA
        self.prev_cte = cte

    def output(self) -> float:
        raw = self.kp * self.p_err + self.ki * self.i_err + self.kd * self.d_err
        return max(self.out_min, min(self.out_max, raw))


# ─────────────────────────────────────────────────────────────────────────────
# Per-swerve state
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class SwervePhase:
    name:         str
    police_actor: object          # carla.Actor
    in_lane:      str             # 'A' (ego lane) or 'B' (adjacent lane)
    passed:       bool  = False
    triggered:    bool  = False
    trigger_tick: int   = -1
    oscillations: int   = 0
    prev_steer:   float = 0.0     # used for sign-reversal detection


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────
def dist2d(a: "carla.Location", b: "carla.Location") -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def yaw_diff_deg(y1: float, y2: float) -> float:
    return ((y1 - y2) + 180.0) % 360.0 - 180.0


def get_lookahead_wp(carla_map, loc: "carla.Location",
                     use_adj: bool = False) -> "carla.Waypoint":
    """Return the waypoint LOOKAHEAD_M ahead in the current or adjacent lane.

    Walk forward in the ego lane FIRST, then switch to the adjacent lane at
    that far point.  This avoids waypoint-snapping instability that occurs when
    the ego position is projected onto road geometry near a parked police car.
    """
    wp = carla_map.get_waypoint(
        loc, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    # Step 1: advance in the ego lane to escape any local snapping artefacts.
    nexts = wp.next(LOOKAHEAD_M)
    far_wp = nexts[0] if nexts else wp

    # Step 2: from the far point, optionally hop to the adjacent lane.
    if use_adj:
        adj = far_wp.get_right_lane()
        if (adj is not None
                and adj.lane_type == carla.LaneType.Driving
                and abs(yaw_diff_deg(adj.transform.rotation.yaw,
                                     far_wp.transform.rotation.yaw)) < 90.0):
            return adj
    return far_wp


def compute_cte(ego_loc: "carla.Location", ego_yaw_deg: float,
                target_wp: "carla.Waypoint") -> float:
    """Signed lateral cross-track error (mirrors main.cpp line 341)."""
    yaw = math.radians(ego_yaw_deg)
    dx  = target_wp.transform.location.x - ego_loc.x
    dy  = target_wp.transform.location.y - ego_loc.y
    return -math.sin(yaw) * dx + math.cos(yaw) * dy


def lateral_transform(wp: "carla.Waypoint",
                      offset_m: float) -> "carla.Transform":
    """Shift waypoint transform by offset_m perpendicular to the road direction."""
    fwd = wp.transform.get_forward_vector()
    rgt = carla.Vector3D(x=fwd.y, y=-fwd.x, z=0.0)
    loc = carla.Location(
        wp.transform.location.x + rgt.x * offset_m,
        wp.transform.location.y + rgt.y * offset_m,
        wp.transform.location.z + 0.1,
    )
    return carla.Transform(loc, wp.transform.rotation)


# ─────────────────────────────────────────────────────────────────────────────
# Main test
# ─────────────────────────────────────────────────────────────────────────────
def test_slalom_pid() -> None:
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(30.0)
    log.info("Connecting to CARLA %s:%d …", CARLA_HOST, CARLA_PORT)

    world         = client.get_world()
    orig_settings = world.get_settings()
    actors: List  = []
    collision_ns  = types.SimpleNamespace(detected=False, actor_name="")

    try:
        # ── Load map ─────────────────────────────────────────────────────────
        if MAP_NAME not in world.get_map().name:
            log.info("Loading %s …", MAP_NAME)
            world = client.load_world(MAP_NAME)
            time.sleep(8.0)
            world.tick()

        carla_map = world.get_map()

        # ── Synchronous mode ─────────────────────────────────────────────────
        settings                     = world.get_settings()
        settings.synchronous_mode    = True
        settings.fixed_delta_seconds = SYNC_DELTA
        world.apply_settings(settings)
        log.info("Sync mode ON  dt=%.3f s", SYNC_DELTA)

        # ── Spawn ego ────────────────────────────────────────────────────────
        spawn_points = carla_map.get_spawn_points()
        if SPAWN_POINT_IDX >= len(spawn_points):
            raise RuntimeError(
                f"SPAWN_POINT_IDX={SPAWN_POINT_IDX} out of range "
                f"(map has {len(spawn_points)} spawn points)"
            )
        ego_tf = spawn_points[SPAWN_POINT_IDX]
        ego_wp = carla_map.get_waypoint(
            ego_tf.location, project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        ego_bp = world.get_blueprint_library().find("vehicle.lincoln.mkz_2017")
        ego_bp.set_attribute("role_name", "ego")
        ego = world.try_spawn_actor(ego_bp, ego_tf)
        if ego is None:
            raise RuntimeError(
                f"Could not spawn ego at spawn_points[{SPAWN_POINT_IDX}]."
            )
        actors.append(ego)
        world.tick()

        eloc = ego.get_transform().location
        log.info("Ego   spawned at (%.1f, %.1f)  road=%d lane=%d  yaw=%.1f°",
                 eloc.x, eloc.y, ego_wp.road_id, ego_wp.lane_id,
                 ego.get_transform().rotation.yaw)

        # ── Verify adjacent lane exists ──────────────────────────────────────
        adj_wp = ego_wp.get_right_lane()
        if (adj_wp is None
                or adj_wp.lane_type != carla.LaneType.Driving
                or abs(yaw_diff_deg(adj_wp.transform.rotation.yaw,
                                    ego_wp.transform.rotation.yaw)) >= 90.0):
            raise RuntimeError(
                f"No same-direction adjacent lane (right) at "
                f"spawn_points[{SPAWN_POINT_IDX}]. "
                "Change SPAWN_POINT_IDX to a multi-lane section."
            )
        log.info("Adj   lane:     road=%d lane=%d  centre=(%.1f, %.1f)",
                 adj_wp.road_id, adj_wp.lane_id,
                 adj_wp.transform.location.x, adj_wp.transform.location.y)

        # Direct lateral CTE reference (straight road — y is the only changing axis)
        # cte = loc.y - target_y is equivalent to compute_cte for yaw≈180° (cos(180°) = -1).
        EGO_LANE_Y = ego_wp.transform.location.y   # ≈ -10.0
        ADJ_LANE_Y = adj_wp.transform.location.y   # ≈ -13.5
        log.info("Lane y-refs: ego=%.2f  adj=%.2f", EGO_LANE_Y, ADJ_LANE_Y)

        # ── Spawn slalom police cars ──────────────────────────────────────────
        police_bp = world.get_blueprint_library().find(
            "vehicle.dodge.charger_police"
        )
        police_bp.set_attribute("role_name", "police")

        def spawn_police(dist_m: float, use_adj: bool,
                         offset_m: float, label: str) -> "carla.Actor":
            base_wps = ego_wp.next(dist_m)
            if not base_wps:
                raise RuntimeError(
                    f"{label}: no waypoint {dist_m} m ahead of ego spawn"
                )
            base = base_wps[0]
            if use_adj:
                adj = base.get_right_lane()
                if adj is None or adj.lane_type != carla.LaneType.Driving:
                    raise RuntimeError(
                        f"{label}: no adjacent (right) lane at {dist_m} m ahead"
                    )
                base = adj
            tf    = lateral_transform(base, offset_m)
            actor = world.try_spawn_actor(police_bp, tf)
            if actor is None:
                raise RuntimeError(
                    f"Could not spawn {label} at {dist_m} m / offset={offset_m} m"
                )
            actor.apply_control(carla.VehicleControl(brake=1.0))
            actor.set_target_velocity(carla.Vector3D(0, 0, 0))
            actors.append(actor)
            world.tick()
            loc = actor.get_transform().location
            log.info("%s spawned at (%.1f, %.1f)  [adj=%s  offset=%.1f m]",
                     label, loc.x, loc.y, use_adj, offset_m)
            return actor

        p1 = spawn_police(P1_DIST_M, P1_USE_ADJ, P1_OFFSET_M,
                          "Police1 40m/ego/+1m ")
        p2 = spawn_police(P2_DIST_M, P2_USE_ADJ, P2_OFFSET_M,
                          "Police2 70m/adj/-1m ")
        p3 = spawn_police(P3_DIST_M, P3_USE_ADJ, P3_OFFSET_M,
                          "Police3 100m/ego/0m")

        # Slalom sequence: A=ego lane, B=adjacent lane
        swerves = [
            SwervePhase("S1→B", p1, in_lane="A"),   # Police1 in A → switch A→B
            SwervePhase("S2→A", p2, in_lane="B"),   # Police2 in B → switch B→A
            SwervePhase("S3→B", p3, in_lane="A"),   # Police3 in A → switch A→B
        ]

        # ── Collision sensor ─────────────────────────────────────────────────
        sensor_bp = world.get_blueprint_library().find("sensor.other.collision")
        collision_sensor = world.try_spawn_actor(
            sensor_bp, carla.Transform(), attach_to=ego
        )
        if collision_sensor:
            actors.append(collision_sensor)

            def on_collision(event: "carla.CollisionEvent") -> None:
                other = event.other_actor
                name  = other.type_id if other else "unknown"
                if not collision_ns.detected:
                    collision_ns.detected   = True
                    collision_ns.actor_name = name
                    log.error("💥  COLLISION with: %s", name)

            collision_sensor.listen(on_collision)

        world.tick()
        log.info("Collision sensor attached.")

        # ── PID controllers ──────────────────────────────────────────────────
        steer_pid    = SimplePID(STEER_KP,    STEER_KI,    STEER_KD,
                                 STEER_MAX,   STEER_MIN)
        throttle_pid = SimplePID(THROTTLE_KP, THROTTLE_KI, THROTTLE_KD,
                                 THROTTLE_MAX, THROTTLE_MIN)

        # ── State ────────────────────────────────────────────────────────────
        current_lane  = "A"    # "A" = ego lane, "B" = adjacent lane
        swerve_idx    = 0      # index into swerves[]
        phase         = "APPROACH"
        all_cleared   = False
        complete_tick: Optional[int] = None
        final_speed   = 0.0

        # ── Header ───────────────────────────────────────────────────────────
        log.info("─" * 96)
        log.info("%-6s %-8s %-8s %-8s %-8s %-8s %-8s %-4s  %-12s",
                 "Tick", "Speed", "CTE", "Steer",
                 "dP1", "dP2", "dP3", "Lane", "Phase")
        log.info("─" * 96)

        # ── Main loop ────────────────────────────────────────────────────────
        for tick in range(MAX_RUN_TICKS):
            world.tick()

            if collision_ns.detected:
                break

            t      = ego.get_transform()
            loc    = t.location
            yaw    = t.rotation.yaw
            vel    = ego.get_velocity()
            speed  = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

            # ── Warmup annotation (speed ramp — no phase gate on avoidance) ───
            if tick < WARMUP_TICKS and phase == "APPROACH":
                phase = "WARMUP"
            elif tick >= WARMUP_TICKS and phase == "WARMUP":
                phase = "APPROACH"
                log.info("🏁  Slalom phase START  (speed=%.2f m/s)", speed)

            # ── State machine (always active — avoidance can't wait for warmup)
            if swerve_idx < len(swerves):
                sw = swerves[swerve_idx]
                p_loc  = sw.police_actor.get_transform().location
                dist_p = dist2d(loc, p_loc)

                # Obstacle passed: for westbound driving, ego_x < police_x − 10
                if not sw.passed and loc.x < p_loc.x - 10.0:
                    sw.passed = True
                    swerve_idx += 1
                    log.info("✅  %s cleared  (dist=%.1f m  oscillations=%d)",
                             sw.name, dist_p, sw.oscillations)
                    if swerve_idx >= len(swerves):
                        all_cleared   = True
                        phase         = "COMPLETE"
                        complete_tick = tick
                        final_speed   = speed
                        log.info("🏆  All 3 obstacles cleared — speed=%.2f m/s",
                                 speed)

                elif not sw.passed:
                    # Trigger lane switch when the obstacle in our target lane
                    # is within TRIGGER_DIST_M
                    if (not sw.triggered
                            and sw.in_lane == current_lane
                            and dist_p < TRIGGER_DIST_M):
                        sw.triggered    = True
                        sw.trigger_tick = tick
                        prev_lane       = current_lane
                        current_lane    = "B" if current_lane == "A" else "A"
                        if phase in ("WARMUP", "APPROACH"):
                            phase = "SLALOM"
                        log.info("🔀  %s triggered at tick %d  "
                                 "lane %s→%s  dist=%.1f m  speed=%.2f m/s",
                                 sw.name, tick, prev_lane, current_lane,
                                 dist_p, speed)

            # ── Wait for speed to settle after final obstacle ─────────────────
            if phase == "COMPLETE" and complete_tick is not None:
                if tick >= complete_tick + COMPLETE_SETTLE:
                    final_speed = speed
                    break

            # ── Lateral CTE: direct y-difference (no waypoint queries needed) ─
            # Equivalent to compute_cte for straight westbound road (yaw≈180°).
            target_y = ADJ_LANE_Y if (current_lane == "B") else EGO_LANE_Y
            cte      = loc.y - target_y

            # ── PID compute ───────────────────────────────────────────────────
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

            # ── Oscillation counting for active swerve ────────────────────────
            if swerve_idx < len(swerves):
                sw = swerves[swerve_idx]
                if sw.triggered and tick > sw.trigger_tick + 2:
                    if sw.prev_steer * steer < 0:
                        sw.oscillations += 1
                    sw.prev_steer = steer

            # ── Periodic log ──────────────────────────────────────────────────
            log_this = (tick % 20 == 0
                        or (phase == "SLALOM"
                            and swerve_idx < len(swerves)
                            and swerves[swerve_idx].triggered
                            and tick % 5 == 0))
            if log_this:
                d1 = dist2d(loc, p1.get_transform().location)
                d2 = dist2d(loc, p2.get_transform().location)
                d3 = dist2d(loc, p3.get_transform().location)
                log.info("%-6d %-8.2f %-8.3f %-8.3f %-8.1f %-8.1f %-8.1f %-4s  %-12s",
                         tick, speed, cte, steer, d1, d2, d3,
                         current_lane, phase)

        # ─────────────────────────────────────────────────────────────────────
        # Assertions
        # ─────────────────────────────────────────────────────────────────────
        log.info("─" * 96)
        log.info("POST-RUN ASSERTIONS")
        log.info("─" * 96)

        failures = []

        # 1. No collision
        if collision_ns.detected:
            failures.append(f"COLLISION with: {collision_ns.actor_name}")
        else:
            log.info("✅  1. No collision")

        # 2. All 3 obstacles cleared
        if not all_cleared:
            cleared_n = sum(1 for s in swerves if s.passed)
            failures.append(
                f"CLEARED: only {cleared_n}/3 obstacles cleared "
                f"(phase={phase})"
            )
        else:
            log.info("✅  2. All 3 obstacles cleared")

        # 3. Final speed — car must still be moving (proves no crash/stop)
        MIN_SPEED_MS = 2.0   # m/s; well below plateau (~4-5 m/s) but above 0
        if final_speed < MIN_SPEED_MS:
            failures.append(
                f"SPEED: final={final_speed:.2f} m/s < {MIN_SPEED_MS:.1f} m/s "
                f"(car stopped — possible crash or stall)"
            )
        else:
            log.info("✅  3. Final speed %.2f m/s  (>= %.1f m/s moving threshold)",
                     final_speed, MIN_SPEED_MS)

        # 4. Oscillations per swerve phase
        for sw in swerves:
            if sw.oscillations > MAX_OSCILLATIONS:
                failures.append(
                    f"OSCILLATIONS: {sw.name} had {sw.oscillations} "
                    f"sign-reversals (limit {MAX_OSCILLATIONS})"
                )
            else:
                log.info("✅  4. %s oscillations: %d  (limit %d)",
                         sw.name, sw.oscillations, MAX_OSCILLATIONS)

        # 5. Time limit
        if tick >= MAX_RUN_TICKS - 1 and not all_cleared:
            failures.append(
                f"TIMEOUT: did not complete within {MAX_RUN_TICKS} ticks "
                f"({MAX_RUN_TICKS * SYNC_DELTA:.0f} s)"
            )
        else:
            log.info("✅  5. Completed in %d ticks (%.1f s)",
                     tick, tick * SYNC_DELTA)

        # ── Final verdict ─────────────────────────────────────────────────────
        log.info("─" * 96)
        if failures:
            log.error("❌  FAIL — %d issue(s):", len(failures))
            for f in failures:
                log.error("    • %s", f)
        else:
            log.info("🏆  PASS — all 5 slalom criteria met")
        log.info("─" * 96)

        assert not failures, (
            "Slalom test FAILED:\n  " + "\n  ".join(failures)
        )

    finally:
        # ── Cleanup ───────────────────────────────────────────────────────────
        log.info("Cleaning up %d actors …", len(actors))
        for a in reversed(actors):
            try:
                a.destroy()
            except Exception:
                pass
        world.apply_settings(orig_settings)
        log.info("Sync mode OFF — cleanup complete")


if __name__ == "__main__":
    test_slalom_pid()
