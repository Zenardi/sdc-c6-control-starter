#!/usr/bin/env bash
# Run once after the container is created.
# Idempotent — safe to re-run.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "==> Creating CARLA RPC stub library..."
CARLA_LIB_DIR="$REPO_ROOT/project/CARLA/CarlaUE4/Plugins/Carla/CarlaDependencies/lib"
mkdir -p "$CARLA_LIB_DIR"
# Create an empty archive so the CMake linker target is satisfied at build time.
# Runtime communication is WebSocket-only (simulatorAPI.py); no real CARLA
# C++ client library is required.
ar rcs "$CARLA_LIB_DIR/librpc.a"

echo "==> Building pid_controller..."
cd "$REPO_ROOT/project/pid_controller"
cmake -S . -B . 2>&1
make -j"$(nproc)" 2>&1

echo ""
echo "==> Running unit tests..."
./tests/pid_test

echo ""
echo "✅  Dev container is ready."
echo "    • Build:  cd project/pid_controller && make -j\$(nproc)"
echo "    • Test:   cd project/pid_controller && ./tests/pid_test"
echo "    • Run:    cd project && ./run_main_pid.sh   (requires CARLA)"
