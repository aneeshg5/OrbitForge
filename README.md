# OrbitForge

**Live spacecraft state estimation — real satellites, real filters, runs in your browser.**

[orbitforge.dev](https://orbitforge.dev) — zero install, works offline after first load.

---

![OrbitForge main view](docs/screenshots/main-view.png)

Load any real satellite's TLE, watch KF / EKF / UKF estimate its position and attitude against simulated sensor noise, inject faults, and see filter divergence in real time — or run full Monte Carlo consistency campaigns with configurable filter, run duration, process noise, and seed. All computation runs in a C++17 WebAssembly engine at near-native speed — no server, no backend.

## What makes this different

| Tool | Gap |
|------|-----|
| Orekit (Java) | CLI/API only, no browser, no filter comparison |
| nyx (Rust WASM) | Propagation only — no state estimation, no sensors |
| orbidet (Python) | Notebook-only, not interactive |
| MATLAB toolbox | Requires license, offline |

OrbitForge is the first tool that combines real TLE data + EKF/UKF filter comparison + fault injection + WASM — all running in the browser.

## Build

### Engine (C++17, native — tests and benchmarks)

```bash
# Requires: cmake, Eigen3, internet (GTest fetched automatically)
cmake -B build -DCMAKE_BUILD_TYPE=Debug engine/
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```

### Web (TypeScript + WASM)

```bash
cd web
npm install
npm run dev      # local dev server with COOP/COEP headers
npm run build    # production build to web/dist/
```

The production build requires the WASM artifacts (`orbitforge.wasm`,
`orbitforge.js`) to have been built first via `scripts/build_wasm.sh`
(Emscripten) — see [docs/architecture.md](docs/architecture.md) for the
full toolchain.

## Architecture

See [docs/architecture.md](docs/architecture.md), [docs/math.md](docs/math.md),
and the build log in [docs/checkpoint.md](docs/checkpoint.md).

## Status

- [x] Phase 1 — C++ engine (EOM, RK4/RK45, KF/EKF/UKF, sensors, tests, benchmarks)
- [x] Phase 2 — WASM build + lock-free memory systems (ring buffer, pool allocator, fault injector, Simulation class, web scaffold)
- [x] Phase 3 — WebGL2 renderer + Monte Carlo UI + fault injection + live TLE feed (5 satellite presets verified end-to-end)
- [x] Phase 4 — Cloudflare Pages deploy workflow (CI builds and uploads the WASM artifact on every push; deploying to orbitforge.dev itself still needs `CLOUDFLARE_API_TOKEN`/`CLOUDFLARE_ACCOUNT_ID` repo secrets, not yet configured — account-side, not a code gap)
- [x] Phase 5 — 6DOF attitude estimation (12-state multiplicative EKF for EKF/UKF, torque-free rigid-body dynamics, gyro + magnetometer sensors; KF stays the intentionally-naive 6-state baseline)

Phase 1: 33/33 tests passing; benchmarks in [docs/benchmarks.md](docs/benchmarks.md) — all metrics 13–32x inside target.
Phase 2: 63/63 tests passing; ring buffer throughput 3.46×10⁸/sec (69× target); WASM compile verified in CI.
Phase 3: WebGL2 Earth/orbit/covariance renderers, Chart.js error/NIS/NEES panels, fault injection controls, and live CelesTrak TLE feed all implemented and verified against real network data — see [docs/checkpoint.md](docs/checkpoint.md). Monte Carlo has since grown from a single "Runs" slider into fully configurable campaigns (filter choice, run duration, process noise, random/fixed seed), with live per-realization progress and consistency charts colored to match the filter that ran.
Phase 5: 12-state MEKF for EKF/UKF, rigid-body Euler's-equation dynamics, gyro/magnetometer measurement models with analytically-derived Jacobians, attitude error/angular-velocity chart metrics, and a true-attitude spacecraft marker rendered on the orbit path.

112 engine tests passing (native, clean under ASan/UBSan and ThreadSanitizer); 8 browser end-to-end tests (Playwright) covering the run/pause/reset and Monte Carlo lifecycle.
