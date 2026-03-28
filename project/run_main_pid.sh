#!/bin/bash
# Run the C++ PID server in the background then launch the Python CARLA bridge.
# The C++ server must be listening on port 4567 before Python connects.
#
# CARLA is launched automatically if not already running, using Vulkan renderer
# and Low quality level to maximise FPS on the RTX 5070.

CARLA_ROOT="$(cd "$(dirname "$0")/CARLA" && pwd)"
CARLA_BIN="$CARLA_ROOT/CarlaUE4.sh"

# ── 1. Start CARLA if not already running ───────────────────────────────────
if ! pgrep -f "CarlaUE4-Linux-Shipping" > /dev/null 2>&1; then
    echo "[run] Starting CARLA with Vulkan + Low quality..."
    "$CARLA_BIN" \
        -vulkan \
        -quality-level=Low \
        -prefernvidia \
        -carla-port=2000 \
        -RenderOffScreen \
        > /tmp/carla_server.log 2>&1 &
    CARLA_PID=$!
    echo "[run] CARLA PID=$CARLA_PID (log: /tmp/carla_server.log)"

    # Wait for CARLA to accept connections (up to 30 s)
    echo -n "[run] Waiting for CARLA port 2000..."
    for i in $(seq 1 60); do
        if ss -tlnp 2>/dev/null | grep -q 2000; then
            echo " ready (${i} x 0.5 s)"
            break
        fi
        sleep 0.5
    done
else
    echo "[run] CARLA already running — skipping launch"
fi

# ── 2. Kill any stale C++ PID controller ────────────────────────────────────
pkill -f pid_controller 2>/dev/null
sleep 0.5

./pid_controller/pid_controller &
CPP_PID=$!
echo "[run] C++ server PID=$CPP_PID"

# Wait until the server is actually listening (up to 10 s)
for i in $(seq 1 20); do
    if ss -tlnp 2>/dev/null | grep -q 4567; then
        echo "[run] port 4567 ready (${i} x 0.5 s)"
        break
    fi
    sleep 0.5
done

python3 simulatorAPI.py

# Clean up C++ server when Python exits
kill $CPP_PID 2>/dev/null
