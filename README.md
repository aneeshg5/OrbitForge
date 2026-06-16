# OrbitForge

**Live spacecraft state estimation — real satellites, real filters, runs in your browser.**

[orbitforge.dev](https://orbitforge.dev) — zero install, works offline after first load.

---

Load any real satellite's TLE, watch KF / EKF / UKF run against simulated sensor noise, inject faults, and see filter divergence in real time. All computation runs in a C++17 WebAssembly engine at near-native speed — no server, no backend.

## What makes this different

| Tool | Gap |
|------|-----|
| Orekit (Java) | CLI/API only, no browser, no filter comparison |
| nyx (Rust WASM) | Propagation only — no state estimation, no sensors |
| orbidet (Python) | Notebook-only, not interactive |
| MATLAB toolbox | Requires license, offline |

OrbitForge is the first tool that combines real TLE data + EKF/UKF filter comparison + fault injection + WASM — all running in the browser.

## Build (Phase 1 — C++ engine, native)

```bash
# Requires: cmake, Eigen3, internet (GTest fetched automatically)
cmake -B build -DCMAKE_BUILD_TYPE=Debug engine/
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```

## Architecture

See [docs/architecture.md](docs/architecture.md), [docs/math.md](docs/math.md),
and the build log in [docs/checkpoint.md](docs/checkpoint.md).

## Status

- [x] Phase 1 — C++ engine (EOM, RK4/RK45, KF/EKF/UKF, sensors, tests, benchmarks)
- [ ] Phase 2 — WASM build + lock-free memory systems
- [ ] Phase 3 — WebGL2 renderer + Monte Carlo UI
- [ ] Phase 4 — Cloudflare Pages deploy

Phase 1: 33/33 tests passing (`cd build && ctest`); benchmarks in
[docs/benchmarks.md](docs/benchmarks.md) — all metrics 13–32x inside target.
