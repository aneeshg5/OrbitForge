#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
ENGINE_DIR="$ROOT_DIR/engine"
OUT_DIR="$ROOT_DIR/web/public"

source /opt/emsdk/emsdk_env.sh

mkdir -p "$OUT_DIR"

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
    -I/path/to/eigen \
    "$ENGINE_DIR/src/dynamics/eom.cpp" \
    "$ENGINE_DIR/src/dynamics/perturbations.cpp" \
    "$ENGINE_DIR/src/filters/kf.cpp" \
    "$ENGINE_DIR/src/filters/ekf.cpp" \
    "$ENGINE_DIR/src/filters/ukf.cpp" \
    "$ENGINE_DIR/src/sensors/magnetometer.cpp" \
    "$ENGINE_DIR/src/monte_carlo/mc_runner.cpp" \
    "$ENGINE_DIR/src/wasm_api.cpp" \
    -o "$OUT_DIR/orbitforge.js"

echo "WASM build complete → $OUT_DIR/orbitforge.{js,wasm}"
