#!/bin/bash
echo "[STERILIS] Waiting for /coverage_server to appear..."
until ros2 lifecycle get /coverage_server 2>/dev/null | grep -q "unconfigured"; do
    echo "[STERILIS] coverage_server not ready yet, retrying..."
    sleep 2
done
echo "[STERILIS] coverage_server ready. Configuring..."
ros2 lifecycle set /coverage_server configure
sleep 2
echo "[STERILIS] Activating..."
ros2 lifecycle set /coverage_server activate
echo "[STERILIS] coverage_server fully active."
