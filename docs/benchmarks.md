# OrbitForge — Benchmark Results

Performance targets and committed measurements. Updated on each tagged release.

## Targets

| Operation | Target |
|-----------|--------|
| Single RK4 step (6-state, J2+drag) | < 2 μs |
| Single EKF step (6-state, GPS meas) | < 8 μs |
| Single UKF step (SR form, 6-state) | < 20 μs |
| 3 filters simultaneously (one tick) | < 40 μs |
| Monte Carlo 1000 runs × 1000 steps | < 800 ms |
| WASM overhead vs native (RK4) | < 20% |
| Ring buffer push/pop throughput | > 5M frames/sec |
| AoS → SoA MC improvement | > 30% |

## Measurements — Phase 1 native build (2026-06-15)

**Environment:** Apple M4 Pro, macOS 26.3.1, Apple clang 17.0.0, CMake Release (`-O2`), native arm64 (no WASM — that is a Phase 2 measurement). 6-state filters, 3-axis GPS measurement, `dt = 10 s`. Built via `scripts/benchmark.sh` → `cmake -B build_rel -DCMAKE_BUILD_TYPE=Release engine/`.

| Operation | Target | Measured | Margin |
|-----------|--------|----------|--------|
| Single RK4 step (6-state, J2+drag) | < 2 μs | **0.149 μs** | 13.4x |
| Single EKF step (predict+update, GPS meas) | < 8 μs | **0.247 μs** | 32.4x |
| Single UKF step (SR form, predict+update) | < 20 μs | **1.292 μs** | 15.5x |
| 3 filters simultaneously (one tick) | < 40 μs | **1.655 μs** | 24.2x |
| Monte Carlo 1000 runs × 1000 steps | < 800 ms | *not measured — Phase 3 (mc_runner.cpp not built)* | — |
| WASM overhead vs native (RK4) | < 20% | *not measured — Phase 2 (no WASM build yet)* | — |
| Ring buffer push/pop throughput | > 5M frames/sec | *not measured — Phase 2 (ring_buffer.hpp not built)* | — |
| AoS → SoA MC improvement | > 30% | *not measured — Phase 3 (ensemble.hpp not built)* | — |

Raw output (`./scripts/benchmark.sh`):
```
compute_acceleration (J2+drag, 1000000 iterations):  0.014 us/call
rk4_step (6-state, J2+drag, 1000000 iterations):     0.149 us/step
KF  predict+update (100000 iterations):              0.138 us/step
EKF predict+update (100000 iterations):              0.247 us/step
UKF predict+update (100000 iterations):               1.292 us/step
3 filters simultaneously (100000 iterations):         1.655 us/tick
```

**Methodology notes** (`engine/benchmarks/bench_integrator.cpp`, `bench_filters.cpp`):
- `compute_acceleration`/`rk4_step`: fixed ISS-like state, J2+drag enabled, `high_resolution_clock`, single accumulator with a `volatile` sink to block dead-code elimination.
- KF/EKF/UKF: filter `predict(dt)` immediately followed by `update(z)` where `z = x̂.head<3>()` (zero-innovation measurement). This keeps the filter numerically stable indefinitely — `x` just follows real two-body+J2 dynamics under `predict()`, and `update()` still exercises the full Kalman gain / covariance pipeline since `P` is non-degenerate. EKF/UKF benchmarks use J2-only perturbations (no drag), matching what their analytical Jacobians/sigma-point propagation actually model.
- "3 filters simultaneously" runs all three filters' predict+update inside one loop iteration — this is the number that bounds achievable tick rate (target supports 100 Hz with 24x headroom).
- All numbers reflect native arm64 performance only. WASM/pthread overhead, Monte Carlo throughput, ring buffer throughput, and SoA-vs-AoS comparisons are deferred to Phase 2/3 once the corresponding components exist, per `docs/checkpoint.md`.
