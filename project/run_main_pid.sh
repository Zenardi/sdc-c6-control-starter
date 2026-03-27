#!/bin/bash
# Run the C++ PID server in the background then launch the Python CARLA bridge.
# The C++ server must be listening on port 4567 before Python connects.

# Kill any stale instances
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
