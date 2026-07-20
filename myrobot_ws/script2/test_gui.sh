#!/bin/bash
# Gazebo GUI test script with VcXsrv support

echo "=== Gazebo GUI Test ==="
echo "Time: $(date)"
echo ""

# Check VcXsrv and set DISPLAY
echo "[1/3] Checking VcXsrv X server..."
HOST_IP="$(ip route show default 2>/dev/null | awk '{print $3; exit}')"
if [ -z "$HOST_IP" ]; then
    echo "  ❌ FAIL: Cannot detect host IP"
    exit 1
fi

if timeout 1 bash -c "echo > /dev/tcp/$HOST_IP/6000" 2>/dev/null; then
    echo "  ✓ OK: VcXsrv found at $HOST_IP:0.0"
    export DISPLAY="$HOST_IP:0.0"
    unset WAYLAND_DISPLAY
else
    echo "  ❌ FAIL: X server not reachable at $HOST_IP:6000"
    echo "  → Please start VcXsrv on Windows first"
    exit 1
fi
echo ""

# Check gazebo installation
echo "[2/3] Checking Gazebo installation..."
if ! which gz > /dev/null 2>&1; then
    echo "  ❌ FAIL: gazebo not found"
    exit 1
fi
GZ_VERSION=$(gz sim --version 2>&1 | head -n1)
echo "  ✓ OK: $GZ_VERSION"
echo ""

# Test GUI
echo "[3/3] Running Gazebo GUI test (12 seconds)..."
LOG="/tmp/gz_gui_test_$$.log"
timeout 12 gz sim -v1 shapes.sdf > "$LOG" 2>&1
EXIT=$?

echo ""
if grep -q "Unable to create the rendering window\|Segmentation fault" "$LOG" 2>/dev/null; then
    echo "  ❌ FAIL: Rendering error detected"
    grep -i "error\|ogre.*exception" "$LOG" | head -n 3
    exit 1
elif [ $EXIT -eq 0 ] || [ $EXIT -eq 124 ]; then
    echo "  ✓ OK: GUI launched successfully"
    if [ $EXIT -eq 124 ]; then
        echo "  (terminated after 12s as expected)"
    fi
else
    echo "  ❌ FAIL: Unexpected exit code $EXIT"
    tail -n 10 "$LOG"
    exit 1
fi

echo ""
echo "=== ✓ All tests passed ==="
echo ""
echo "Ready to run Gazebo GUI:"
echo "  $ export DISPLAY=\$HOSTIP:0.0"
echo "  $ gz sim"
echo ""
echo "Or simply run:"
echo "  $ gz sim"
echo "(DISPLAY is auto-configured in ~/.bashrc if VcXsrv is running)"
