#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cmake -B "$ROOT_DIR/build_rel" -DCMAKE_BUILD_TYPE=Release "$ROOT_DIR/engine"
cmake --build "$ROOT_DIR/build_rel" -j"$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)"

echo "=== OrbitForge Benchmarks ==="
"$ROOT_DIR/build_rel/orbitforge_benchmarks"
