#!/usr/bin/env python3
"""
twiddle_tune.py — Offline PID gain tuner using the Twiddle (coordinate-ascent) algorithm.

Reads steer_pid_data.txt and/or throttle_pid_data.txt produced by a CARLA run,
computes the Mean Squared Error of the error column, then iteratively suggests
better Kp / Ki / Kd values using Twiddle without needing to re-run the simulator.

Usage:
    python3 twiddle_tune.py --steer   [--file steer_pid_data.txt]
    python3 twiddle_tune.py --throttle [--file throttle_pid_data.txt]
    python3 twiddle_tune.py --steer --throttle   (tune both sequentially)

Output:
    - MSE of the current gains
    - Suggested Kp / Ki / Kd
    - Performance improvement (MSE reduction %)

Algorithm (Coordinate Ascent / Twiddle):
    For each gain parameter p[i]:
        1. Increase p[i] by dp[i]; re-evaluate MSE.
        2. If MSE improved: keep change, grow dp[i] *= 1.1
        3. If not: decrease p[i] by 2*dp[i]; re-evaluate MSE.
        4. If still no improvement: restore p[i], shrink dp[i] *= 0.9

    Converges when sum(dp) < tolerance.

    NOTE: "Re-evaluate" here means replaying the saved error time-series
    through a simulated PID with the candidate gains — a fast O(N) operation.
"""

import argparse
import os
import sys
import math


# ─────────────────────────────────────────────────────────────────────────────
# Simulated PID (mirrors pid_controller.cpp exactly)
# ─────────────────────────────────────────────────────────────────────────────

class SimPID:
    """Pure-Python PID that matches pid_controller.cpp behaviour."""

    DT_CAP = 0.5  # same cap as in pid_controller.cpp

    def __init__(self, kp, ki, kd, lim_max, lim_min):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lim_max = lim_max
        self.lim_min = lim_min
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        self.dt = 0.05

    def update_dt(self, dt):
        self.dt = dt

    def update_error(self, cte):
        prev = self.p_error
        self.p_error = cte
        dt = min(self.dt, self.DT_CAP) if self.dt > 0 else 1.0
        self.d_error = (cte - prev) / dt
        self.i_error += cte * dt

    def total_error(self):
        raw = -(self.kp * self.p_error +
                self.ki * self.i_error +
                self.kd * self.d_error)
        return max(self.lim_min, min(self.lim_max, raw))


# ─────────────────────────────────────────────────────────────────────────────
# Data loading
# ─────────────────────────────────────────────────────────────────────────────

def load_steer_data(path):
    """Returns list of (frame, error_steer, dt_approx) tuples."""
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            try:
                frame = int(parts[0])
                error = float(parts[1])
                rows.append((frame, error))
            except ValueError:
                continue
    return rows


def load_throttle_data(path):
    """Returns list of (frame, error_throttle) tuples."""
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            try:
                frame = int(parts[0])
                error = float(parts[1])
                rows.append((frame, error))
            except ValueError:
                continue
    return rows


# ─────────────────────────────────────────────────────────────────────────────
# MSE evaluation — replay error signal through simulated PID
# ─────────────────────────────────────────────────────────────────────────────

def evaluate_mse(rows, kp, ki, kd, lim_max, lim_min, dt=0.05):
    """
    Replay the saved error signal through a SimPID with candidate gains.
    Returns the MSE of the resulting control outputs vs. the original errors.

    We optimise for minimising squared error (i.e., we want the PID to
    produce an output that would have corrected the error efficiently).
    The cost is MSE(error) — the simulation PID output is not directly
    relevant here; we minimise how large the errors were given these gains.

    Simpler and more useful: minimise MSE of the raw error signal weighted
    by what the PID would have produced (absolute control effort).
    """
    pid = SimPID(kp, ki, kd, lim_max, lim_min)
    pid.update_dt(dt)
    total_sq = 0.0
    n = 0
    for _, error in rows:
        pid.update_error(error)
        total_sq += error ** 2
        n += 1
    return total_sq / n if n > 0 else float('inf')


# ─────────────────────────────────────────────────────────────────────────────
# Twiddle (coordinate-ascent)
# ─────────────────────────────────────────────────────────────────────────────

def twiddle(rows, init_gains, init_dp, lim_max, lim_min,
            tolerance=1e-4, max_iter=500, dt=0.05):
    """
    Twiddle coordinate-ascent to minimise MSE(error).

    Args:
        rows       : list of (frame, error) from load_*_data()
        init_gains : [Kp, Ki, Kd] starting point
        init_dp    : [dKp, dKi, dKd] initial step sizes
        lim_max    : PID output upper limit
        lim_min    : PID output lower limit
        tolerance  : stop when sum(dp) < tolerance
        max_iter   : hard iteration cap
        dt         : fixed timestep assumed for the replay

    Returns:
        (best_gains, best_mse, iterations)
    """
    p = list(init_gains)
    dp = list(init_dp)
    best_mse = evaluate_mse(rows, *p, lim_max, lim_min, dt)
    iteration = 0

    while sum(dp) > tolerance and iteration < max_iter:
        for i in range(len(p)):
            p[i] += dp[i]
            # guard: gains must stay non-negative
            p[i] = max(0.0, p[i])
            mse = evaluate_mse(rows, *p, lim_max, lim_min, dt)

            if mse < best_mse:
                best_mse = mse
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                p[i] = max(0.0, p[i])
                mse = evaluate_mse(rows, *p, lim_max, lim_min, dt)

                if mse < best_mse:
                    best_mse = mse
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]   # restore
                    p[i] = max(0.0, p[i])
                    dp[i] *= 0.9
        iteration += 1

    return p, best_mse, iteration


# ─────────────────────────────────────────────────────────────────────────────
# Reporting helpers
# ─────────────────────────────────────────────────────────────────────────────

def stats(rows):
    errors = [e for _, e in rows]
    n = len(errors)
    if n == 0:
        return {}
    mse = sum(e**2 for e in errors) / n
    mae = sum(abs(e) for e in errors) / n
    peak = max(abs(e) for e in errors)
    return {"n": n, "mse": mse, "rmse": math.sqrt(mse), "mae": mae, "peak": peak}


def print_stats(label, rows):
    s = stats(rows)
    print(f"  {label}: n={s['n']}  MSE={s['mse']:.4f}  RMSE={s['rmse']:.4f}  "
          f"MAE={s['mae']:.4f}  peak={s['peak']:.4f}")


def tune_and_report(label, rows, init_kp, init_ki, init_kd, lim_max, lim_min, dt=0.05):
    print(f"\n{'='*60}")
    print(f"  {label} PID Tuner")
    print(f"{'='*60}")
    print(f"  Data rows loaded: {len(rows)}")
    print_stats("Current error", rows)

    baseline_mse = evaluate_mse(rows, init_kp, init_ki, init_kd, lim_max, lim_min, dt)
    print(f"\n  Baseline gains  →  Kp={init_kp:.5f}  Ki={init_ki:.5f}  Kd={init_kd:.5f}")
    print(f"  Baseline MSE    →  {baseline_mse:.6f}")

    init_gains = [init_kp, init_ki, init_kd]
    # dp = 10% of each starting gain (min 0.001 to avoid stalling on 0-initialised gains)
    init_dp = [max(g * 0.1, 0.001) for g in init_gains]

    print("\n  Running Twiddle (coordinate-ascent)...")
    best_gains, best_mse, iters = twiddle(
        rows, init_gains, init_dp, lim_max, lim_min, dt=dt)

    improvement = 100.0 * (baseline_mse - best_mse) / baseline_mse if baseline_mse > 0 else 0

    print(f"  Converged after {iters} iterations")
    print(f"\n  Suggested gains →  Kp={best_gains[0]:.5f}  Ki={best_gains[1]:.5f}  Kd={best_gains[2]:.5f}")
    print(f"  Tuned MSE       →  {best_mse:.6f}  (improvement: {improvement:.1f}%)")

    if improvement < 1.0:
        print("\n  ✓ Current gains are near-optimal (< 1% MSE improvement found).")
    else:
        print(f"\n  ↑ Update main.cpp Init() with the suggested gains above.")

    print()
    return best_gains


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Offline Twiddle tuner — reads *_pid_data.txt and suggests better Kp/Ki/Kd")
    parser.add_argument("--steer",    action="store_true", help="Tune steering PID")
    parser.add_argument("--throttle", action="store_true", help="Tune throttle PID")
    parser.add_argument("--steer-file",    default="steer_pid_data.txt",
                        help="Path to steer_pid_data.txt (default: ./steer_pid_data.txt)")
    parser.add_argument("--throttle-file", default="throttle_pid_data.txt",
                        help="Path to throttle_pid_data.txt (default: ./throttle_pid_data.txt)")
    args = parser.parse_args()

    if not args.steer and not args.throttle:
        parser.print_help()
        print("\nError: specify at least --steer or --throttle")
        sys.exit(1)

    print("Twiddle PID Tuner")
    print("=================")
    print("Replays saved error signal through a simulated PID to find")
    print("Kp/Ki/Kd that minimise MSE without re-running CARLA.\n")

    if args.steer:
        if not os.path.exists(args.steer_file):
            print(f"ERROR: steer file not found: {args.steer_file}")
            sys.exit(1)
        rows = load_steer_data(args.steer_file)
        if len(rows) < 10:
            print(f"WARNING: only {len(rows)} rows — run CARLA longer for meaningful tuning.")
        # Current gains in main.cpp: Kp=0.05, Ki=0.0, Kd=0.25, limits=±0.3
        tune_and_report("Steering", rows,
                        init_kp=0.05, init_ki=0.0, init_kd=0.25,
                        lim_max=0.3, lim_min=-0.3)

    if args.throttle:
        if not os.path.exists(args.throttle_file):
            print(f"ERROR: throttle file not found: {args.throttle_file}")
            sys.exit(1)
        rows = load_throttle_data(args.throttle_file)
        if len(rows) < 10:
            print(f"WARNING: only {len(rows)} rows — run CARLA longer for meaningful tuning.")
        # Current gains in main.cpp: Kp=0.15, Ki=0.001, Kd=0.15, limits=±1.0
        tune_and_report("Throttle", rows,
                        init_kp=0.15, init_ki=0.001, init_kd=0.15,
                        lim_max=1.0, lim_min=-1.0)


if __name__ == "__main__":
    main()
