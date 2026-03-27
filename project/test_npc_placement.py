"""
test_npc_placement.py — Offline geometry test for NPC obstacle placement.

Verifies that each NPC is within the collision-detection range of the ego
vehicle so the motion planner is FORCED to generate an avoidance path.

Run:  python3 project/test_npc_placement.py

All assertions must pass before launching CARLA.
"""
import math

# ── CARLA / planning constants (must match planning_params.h) ──────────────────
CIRCLE_OFFSETS = [-1.0, 1.0, 3.0]   # m forward from ego centre
CIRCLE_RADII   = [1.5,  1.5,  1.5]  # m radius each circle

# 7 lateral offsets the motion planner tries (P_NUM_PATHS=7, P_GOAL_OFFSET=1.0m)
# Paths: centre ± 0, ±1, ±2, ±3  →  [-3, -2, -1, 0, 1, 2, 3] × P_GOAL_OFFSET
PATH_OFFSETS = [-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0]  # m lateral from goal

# ── Spawn[4] geometry in Town06_Opt ──────────────────────────────────────────
SPAWN_X = 600.9
SPAWN_Y = -9.96   # road centre
SPAWN_YAW_DEG = -179.6

yaw = math.radians(SPAWN_YAW_DEG)
fwd = (math.cos(yaw), math.sin(yaw))   # ≈ (-1,  0)
rgt = (-math.sin(yaw), math.cos(yaw))  # ≈ ( 0, -1)

# ── NPC offsets to test ────────────────────────────────────────────────────────
# Format: (forward_offset_m, lateral_right_offset_m)
NPC_OFFSETS = [
    (30,   2.29),   # NPC0 north: corrected for fwd.y drift over 30m → y≈-12.46 (2.5m north)
    (65,  -2.95),   # NPC1 south: corrected for fwd.y drift over 65m → y≈-7.46  (2.5m south)
    (110,  1.73),   # NPC2 north: corrected for fwd.y drift over 110m → y≈-12.46 (2.5m north)
]


def npc_position(offset_x, offset_y):
    """Return (x, y) of NPC given forward and right offsets from spawn[4]."""
    x = SPAWN_X + offset_x * fwd[0] + offset_y * rgt[0]
    y = SPAWN_Y + offset_x * fwd[1] + offset_y * rgt[1]
    return x, y


def circles_overlap(ego_x, ego_y, npc_x, npc_y):
    """
    Return (overlaps, min_distance) for all ego–NPC circle pairs.
    ego_x/y is the trajectory point at which the ego is directly alongside the NPC.
    """
    min_dist = float('inf')
    any_overlap = False
    for off in CIRCLE_OFFSETS:
        for r_ego, r_npc in zip(CIRCLE_RADII, CIRCLE_RADII):
            # Place ego circle along forward direction
            ec_x = ego_x + off * fwd[0]
            ec_y = ego_y + off * fwd[1]
            dist = math.hypot(ec_x - npc_x, ec_y - npc_y)
            min_dist = min(min_dist, dist)
            if dist < r_ego + r_npc:
                any_overlap = True
    return any_overlap, min_dist


def check_avoidance_required(npc_x, npc_y, label):
    """
    For each of the 7 candidate spiral lateral offsets, check whether the
    ego circle at that offset would collide with the NPC.
    At least one spiral must collide (to force the planner to pick another),
    AND at least one spiral must be free (so avoidance is possible).
    """
    colliding = []
    free = []
    for path_off in PATH_OFFSETS:
        ego_y_here = SPAWN_Y + path_off  # lateral offset applied to road centre
        overlaps, _ = circles_overlap(npc_x, ego_y_here, npc_x, npc_y)
        if overlaps:
            colliding.append(path_off)
        else:
            free.append(path_off)
    return colliding, free


# ── Run tests ─────────────────────────────────────────────────────────────────
print("=" * 60)
print("NPC Placement Geometry Test")
print("=" * 60)
print(f"Road centre:    x={SPAWN_X}, y={SPAWN_Y}")
print(f"Forward vector: ({fwd[0]:.3f}, {fwd[1]:.3f})")
print(f"Right vector:   ({rgt[0]:.3f}, {rgt[1]:.3f})")
print(f"Collision threshold: {CIRCLE_RADII[0] + CIRCLE_RADII[0]:.1f} m (r+r)")
print()

all_pass = True

for i, (off_x, off_y) in enumerate(NPC_OFFSETS):
    nx, ny = npc_position(off_x, off_y)
    lateral = abs(ny - SPAWN_Y)
    threshold = CIRCLE_RADII[0] + CIRCLE_RADII[0]  # 3.0 m

    in_collision_range = lateral < threshold

    colliding_paths, free_paths = check_avoidance_required(nx, ny, f"NPC{i}")

    # Assertions
    has_blocked_path  = len(colliding_paths) > 0
    has_free_path     = len(free_paths) > 0

    status = "✅ PASS" if (in_collision_range and has_blocked_path and has_free_path) else "❌ FAIL"
    if not (in_collision_range and has_blocked_path and has_free_path):
        all_pass = False

    print(f"NPC{i}  x={nx:.2f}  y={ny:.2f}  (offset fwd={off_x}m  rgt={off_y}m)")
    print(f"  Lateral from road centre : {lateral:.2f} m  (threshold {threshold:.1f} m)")
    print(f"  In collision range       : {'YES' if in_collision_range else 'NO  <-- FIX NEEDED'}")
    print(f"  Spirals blocked          : {colliding_paths}  ({len(colliding_paths)}/7)")
    print(f"  Spirals free             : {free_paths}  ({len(free_paths)}/7)")
    print(f"  Avoidance required+possible: {'YES' if (has_blocked_path and has_free_path) else 'NO  <-- PROBLEM'}")
    print(f"  Result: {status}")
    print()

print("=" * 60)
if all_pass:
    print("ALL TESTS PASSED — safe to launch CARLA with these NPC offsets.")
else:
    print("SOME TESTS FAILED — adjust NPC_OFFSETS before launching CARLA!")
print("=" * 60)

assert all_pass, "NPC placement geometry test failed — see output above."
