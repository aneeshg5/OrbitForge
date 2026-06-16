#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
ENGINE_DIR="$ROOT_DIR/engine"
OUT_DIR="$ROOT_DIR/web/public"

source /opt/emsdk/emsdk_env.sh

mkdir -p "$OUT_DIR"

# Eigen is header-only — any compiler (including em++) can consume the
# system-installed headers directly, no WASM-specific build needed. Resolve
# the same candidate paths native builds use (Homebrew on macOS, apt on
# Ubuntu CI) rather than hardcoding one path; override with EIGEN_INCLUDE if
# neither matches.
if [ -z "${EIGEN_INCLUDE:-}" ]; then
    for candidate in /usr/include/eigen3 /opt/homebrew/include/eigen3 /usr/local/include/eigen3; do
        if [ -d "$candidate" ]; then
            EIGEN_INCLUDE="$candidate"
            break
        fi
    done
fi
if [ -z "${EIGEN_INCLUDE:-}" ]; then
    echo "ERROR: Eigen3 headers not found. Set EIGEN_INCLUDE to the eigen3 include directory." >&2
    exit 1
fi

# mc_runner.cpp (Phase 3, Monte Carlo) doesn't exist yet — add it back to
# this list once engine/src/monte_carlo/mc_runner.cpp is built.
em++ \
    -std=c++17 \
    -O3 \
    -pthread \
    -msimd128 \
    -sUSE_PTHREADS=1 \
    -sPTHREAD_POOL_SIZE=4 \
    -sINITIAL_MEMORY=67108864 \
    -sALLOW_MEMORY_GROWTH=1 \
    -sENVIRONMENT='web,worker' \
    -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","setValue","getValue"]' \
    -I"$ENGINE_DIR/include" \
    -I"$EIGEN_INCLUDE" \
    "$ENGINE_DIR/src/dynamics/eom.cpp" \
    "$ENGINE_DIR/src/dynamics/perturbations.cpp" \
    "$ENGINE_DIR/src/filters/kf.cpp" \
    "$ENGINE_DIR/src/filters/ekf.cpp" \
    "$ENGINE_DIR/src/filters/ukf.cpp" \
    "$ENGINE_DIR/src/sensors/magnetometer.cpp" \
    "$ENGINE_DIR/src/wasm_api.cpp" \
    -o "$OUT_DIR/orbitforge.js"

echo "WASM build complete → $OUT_DIR/orbitforge.{js,wasm}"
