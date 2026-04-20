#!/usr/bin/env bash
# TorqueForge — Unified Presentation Launcher
# Runs every demo in sequence, fullscreen, one after another.
# Usage:  ./run_presentation.sh [--no-rebuild]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

# ── NVIDIA prime offload (safe no-op on non-hybrid or Windows WSL) ────────────
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# ── Optional rebuild ──────────────────────────────────────────────────────────
if [[ "${1:-}" != "--no-rebuild" ]]; then
    echo "==> Building TorqueForge (Release)…"
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
          -G Ninja 2>/dev/null \
          || cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_DIR" --parallel
    echo "==> Build complete."
fi

DEMOS="$BUILD_DIR/bin/demos"

# ── Helper: run a demo, wait for it to exit ───────────────────────────────────
run_demo() {
    local label="$1"
    local cmd="$2"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  DEMO: $label"
    echo "  (Close the window or press ESC to advance to the next demo)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    sleep 1
    cd "$SCRIPT_DIR"   # renderer config JSONs are resolved from CWD
    eval "$cmd" || true
}

# ── Demo sequence ─────────────────────────────────────────────────────────────

# 1. Wrecking-ball (free rigid bodies, impulse-based contact)
run_demo "Wrecking Ball — Impulse Contact & Rigid Bodies" \
    "\"$DEMOS/demo_load_scene\""

# 2. Scissorlift (closed kinematic chain, loop-closure constraints)
run_demo "Scissorlift — Closed Kinematic Chain / Loop Closure" \
    "echo 0 | \"$DEMOS/demo_load_scene_articulated\""

# 3. Spring (prismatic joint, spring-damper)
run_demo "Spring — Prismatic Joint & Spring-Damper" \
    "echo 1 | \"$DEMOS/demo_load_scene_articulated\""

# 4. Spherical joint chain
run_demo "Spherical Joint Chain — 3-DOF Joints" \
    "echo 2 | \"$DEMOS/demo_load_scene_articulated\""

# 5. Spring scale (balancing masses on a lever)
run_demo "Spring Scale — Torque Balance" \
    "\"$DEMOS/demo_spring_scale\""

# 6. Chain pendulum (N-link scaling benchmark for Featherstone dynamics)
run_demo "Chain Pendulum — Scaling Benchmark (30 links)" \
    "\"$DEMOS/demo_chain_pendulum\" 30"

echo ""
echo "==> Presentation complete."
