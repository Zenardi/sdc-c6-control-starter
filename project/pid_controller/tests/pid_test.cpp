/**
 * Standalone PID unit tests — no CARLA / CMake dependencies.
 *
 * Compile:
 *   g++ -std=c++14 -I.. pid_test.cpp ../pid_controller.cpp -o pid_test
 * Run:
 *   ./pid_test
 *
 * Each test verifies one property of the PID controller in isolation.
 * All asserts must pass for the implementation to be mathematically sound
 * before integrating with the CARLA simulator.
 */

#include <cassert>
#include <cmath>
#include <cstdio>
#include "../pid_controller.h"

// Floating-point equality with a small epsilon
static bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) < eps;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: Proportional-only response
//
// Config: Kp=1, Ki=0, Kd=0, limits=[-10, 10], dt=1 s
// Action: UpdateError(2.0)
// Expected: TotalError() = -(Kp * p_error) = -(1 * 2) = -2.0
//   (negative because the controller opposes the error)
// ─────────────────────────────────────────────────────────────────────────────
void test_proportional() {
    PID pid;
    pid.Init(1.0, 0.0, 0.0, 10.0, -10.0);
    pid.UpdateDeltaTime(1.0);
    pid.UpdateError(2.0);
    double output = pid.TotalError();
    assert(near(output, -2.0) && "FAIL test_proportional: expected -2.0");
    printf("  PASS test_proportional: output=%.6f (expected -2.0)\n", output);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: Integral accumulation over multiple steps
//
// Config: Kp=0, Ki=1, Kd=0, limits=[-10, 10], dt=0.5 s
// Action: three calls with cte=1.0
//   i_error after 3 calls = sum(cte * dt) = 3 * 1.0 * 0.5 = 1.5
// Expected: TotalError() = -(Ki * i_error) = -(1 * 1.5) = -1.5
// ─────────────────────────────────────────────────────────────────────────────
void test_integral() {
    PID pid;
    pid.Init(0.0, 1.0, 0.0, 10.0, -10.0);
    pid.UpdateDeltaTime(0.5);
    pid.UpdateError(1.0);
    pid.UpdateError(1.0);
    pid.UpdateError(1.0);
    double output = pid.TotalError();
    assert(near(output, -1.5) && "FAIL test_integral: expected -1.5");
    printf("  PASS test_integral: output=%.6f (expected -1.5)\n", output);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: Derivative response to rate of change
//
// Config: Kp=0, Ki=0, Kd=1, limits=[-20, 20], dt=0.5 s
// Note: pid_controller.cpp caps dt at 0.5 s (to guard against large physics
//       steps). So dt=0.5 is the max usable dt for this test.
// Step A: UpdateError(0.0) → d_error = (0 - 0) / 0.5 = 0
// Step B: UpdateError(3.0) → d_error = (3 - 0) / 0.5 = 6
// Expected after step B: TotalError() = -(Kd * d_error) = -(1 * 6) = -6.0
// ─────────────────────────────────────────────────────────────────────────────
void test_derivative() {
    PID pid;
    pid.Init(0.0, 0.0, 1.0, 20.0, -20.0);
    pid.UpdateDeltaTime(0.5);   // dt at the cap boundary
    pid.UpdateError(0.0);        // seed p_error = 0
    pid.UpdateError(3.0);        // d = (3-0)/0.5 = 6
    double output = pid.TotalError();
    assert(near(output, -6.0) && "FAIL test_derivative: expected -6.0");
    printf("  PASS test_derivative: output=%.6f (expected -6.0)\n", output);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: Output clamping (upper and lower)
//
// Upper clamp: Kp=10, cte=5 → raw=-50 → clamped to lim_min=-1
// Lower clamp: Kp=10, cte=-5 → raw=+50 → clamped to lim_max=+1
// ─────────────────────────────────────────────────────────────────────────────
void test_clamp() {
    {
        PID pid;
        pid.Init(10.0, 0.0, 0.0, 1.0, -1.0);
        pid.UpdateDeltaTime(1.0);
        pid.UpdateError(5.0);
        double output = pid.TotalError();
        assert(near(output, -1.0) && "FAIL test_clamp_lower: expected -1.0 (clamped)");
        printf("  PASS test_clamp_lower: output=%.6f (expected -1.0)\n", output);
    }
    {
        PID pid;
        pid.Init(10.0, 0.0, 0.0, 1.0, -1.0);
        pid.UpdateDeltaTime(1.0);
        pid.UpdateError(-5.0);
        double output = pid.TotalError();
        assert(near(output, 1.0) && "FAIL test_clamp_upper: expected +1.0 (clamped)");
        printf("  PASS test_clamp_upper: output=%.6f (expected +1.0)\n", output);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: Combined PID with realistic steer gains (smoke test)
//
// Mirrors main.cpp steer gains: Kp=0.05, Ki=0.0, Kd=0.25, limits=[-0.3, 0.3]
// With dt=0.05 s and cte=0.5 m:
//   p = 0.05 * 0.5 = 0.025
//   d = 0.25 * (0.5/0.05) = 2.5  → output would be -2.525 → clamped to -0.3
// Two-step: first cte=0, then cte=0.5 to get a clean derivative
// ─────────────────────────────────────────────────────────────────────────────
void test_steer_gains_smoke() {
    PID pid;
    pid.Init(0.05, 0.0, 0.25, 0.3, -0.3);
    pid.UpdateDeltaTime(0.05);
    pid.UpdateError(0.0);   // seed
    pid.UpdateError(0.5);   // step

    // p_term = 0.05*0.5 = 0.025; d_term = 0.25*(0.5-0)/0.05 = 2.5 → raw = -2.525 → clamped -0.3
    double output = pid.TotalError();
    assert(output <= 0.3 && output >= -0.3 && "FAIL test_steer_gains_smoke: output out of bounds");
    assert(output < 0.0 && "FAIL test_steer_gains_smoke: expected negative steer for positive CTE");
    printf("  PASS test_steer_gains_smoke: output=%.6f (expected in [-0.3, 0), clamped)\n", output);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6: Combined PID with realistic throttle gains (smoke test)
//
// Mirrors main.cpp throttle gains: Kp=0.15, Ki=0.001, Kd=0.15, limits=[-1, 1]
// Velocity error = velocity - v_desired; negative error → car is too slow → throttle
// cte = -1.0 (car is 1 m/s below target) → expected positive throttle output
// ─────────────────────────────────────────────────────────────────────────────
void test_throttle_gains_smoke() {
    PID pid;
    pid.Init(0.15, 0.001, 0.15, 1.0, -1.0);
    pid.UpdateDeltaTime(0.1);
    pid.UpdateError(-1.0);  // car is 1 m/s below target → negative cte → positive output

    double output = pid.TotalError();
    assert(output > 0.0 && "FAIL test_throttle_gains_smoke: expected positive throttle");
    assert(output <= 1.0 && output >= -1.0 && "FAIL test_throttle_gains_smoke: output out of bounds");
    printf("  PASS test_throttle_gains_smoke: output=%.6f (expected positive throttle)\n", output);
}

int main() {
    printf("=== PID Unit Tests ===\n\n");

    printf("[1] Proportional term\n");
    test_proportional();

    printf("[2] Integral accumulation\n");
    test_integral();

    printf("[3] Derivative rate-of-change\n");
    test_derivative();

    printf("[4] Output clamping\n");
    test_clamp();

    printf("[5] Steer gains smoke test (Kp=0.05, Kd=0.25, lim=0.3)\n");
    test_steer_gains_smoke();

    printf("[6] Throttle gains smoke test (Kp=0.15, Ki=0.001, Kd=0.15, lim=1.0)\n");
    test_throttle_gains_smoke();

    printf("\n=== All tests passed ===\n");
    return 0;
}
