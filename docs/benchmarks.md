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
| Monte Carlo 1000 runs × 1000 steps | < 800 ms | **141.6 ms** (EKF, 4 threads) | 5.7x |
| WASM overhead vs native (RK4) | < 20% | *not measured — no local Emscripten toolchain this session; deferred to CI, see docs/checkpoint.md* | — |
| Ring buffer push/pop throughput | > 5M frames/sec | **5.74×10⁸ /sec** (single-thread push+pop pairs) | 115x |
| AoS → SoA MC improvement | > 30% | **+484.9%** (SoA + batched kernel) — see note below | 16x |

**Ring buffer measurement (2026-06-16, same environment as above):** single-threaded back-to-back `push()`+`pop()` pairs, 10,000,000 iterations — see `bench_ring_buffer_throughput` in `engine/benchmarks/bench_integrator.cpp`. This measures the raw atomic-operation cost, not real producer/consumer thread-scheduling overhead (a different, much noisier number); the multi-threaded correctness test (`RingBuffer.MultiThreadProducerConsumerNoLoss`, 10M items, TSan-clean) is in `engine/tests/test_ring_buffer.cpp`.

Raw output (`./scripts/benchmark.sh`):
```
compute_acceleration (J2+drag, 1000000 iterations):  0.009 us/call
rk4_step (6-state, J2+drag, 1000000 iterations):     0.136 us/step
KF  predict+update (100000 iterations):        0.133 us/step
EKF predict+update (100000 iterations):        0.239 us/step
UKF predict+update (100000 iterations):        1.265 us/step
3 filters simultaneously (100000 iterations):  1.635 us/tick
ring buffer push+pop pairs/sec (single-thread, 10000000 iterations): 5.74e+08 /sec
Monte Carlo ensemble RK4 batch (N=1000 runs, 100 steps):
  AoS, scalar per-run:                 2.033e+07 ensemble-steps/sec
  SoA, scalar per-run (step_ensemble):  2.285e+07 ensemble-steps/sec  (12.4% vs AoS)
  SoA, batched kernel (step_ensemble_fast): 1.189e+08 ensemble-steps/sec  (484.9% vs AoS)
mc_runner full campaign (N=1000 runs x 1000 steps, EKF, 4 threads): 141.6 ms
```

**Methodology notes** (`engine/benchmarks/bench_integrator.cpp`, `bench_filters.cpp`):
- `compute_acceleration`/`rk4_step`: fixed ISS-like state, J2+drag enabled, `high_resolution_clock`, single accumulator with a `volatile` sink to block dead-code elimination.
- KF/EKF/UKF: filter `predict(dt)` immediately followed by `update(z)` where `z = x̂.head<3>()` (zero-innovation measurement). This keeps the filter numerically stable indefinitely — `x` just follows real two-body+J2 dynamics under `predict()`, and `update()` still exercises the full Kalman gain / covariance pipeline since `P` is non-degenerate. EKF/UKF benchmarks use J2-only perturbations (no drag), matching what their analytical Jacobians/sigma-point propagation actually model.
- "3 filters simultaneously" runs all three filters' predict+update inside one loop iteration — this is the number that bounds achievable tick rate (target supports 100 Hz with 24x headroom).
- All numbers reflect native arm64 performance only. WASM/pthread overhead is deferred to CI (no local Emscripten toolchain this session).

**AoS → SoA Monte Carlo measurement (2026-06-16, `engine/benchmarks/bench_monte_carlo.cpp`, N=1000 runs × 100 RK4 steps, gravity+J2 ISS-like orbit, Release build, same environment as above) — three configurations, not two:**
1. *AoS, scalar per-run:* a `{px,py,pz,vx,vy,vz,cov[36]}` struct per run (the illustrative layout from `CLAUDE.md §7`), each run's RK4 step computed independently via the existing `rk4_step()`/`compute_acceleration()` path.
2. *SoA, scalar per-run (`step_ensemble`):* identical per-run computation, but state stored as six parallel `double[N]` arrays instead of the AoS struct above.
3. *SoA, batched kernel (`step_ensemble_fast`):* SoA storage **and** a batched gravity+J2 acceleration kernel (`accel_gravity_j2_batch`) that operates on the whole `double[N]` array per call, with no per-run `Eigen::Matrix` construction or branch inside the loop — the form that is actually auto-vectorizable.

Configuration 2 vs. 1 showed only **+10–20%** across repeated runs (noisy, close to measurement floor) — *not* the ~40% figure originally estimated in `CLAUDE.md §7`. The reason: at N=1000, both layouts' working sets fit comfortably in L2 cache, and the bottleneck is the floating-point latency inside `compute_acceleration` (sqrt + divides for gravity and J2), which is identical in both layouts since both eventually call the same scalar Eigen code per run. SoA storage alone does not move a compute-bound loop.

Configuration 3 vs. 1 showed **+430–485%** consistently across repeated runs. This is the number that matches the architecture's actual claim in `CLAUDE.md §7` ("cache-prefetchable, **SIMD-vectorizable**") — the gain requires *both* the SoA layout *and* a kernel written so the compiler can pack iterations into SIMD lanes, not the layout by itself. `step_ensemble_fast()` is verified bit-tolerance-identical (`isApprox`, 1e-9) to the generic per-run path in `test_ensemble.cpp`, so this is the same physics, reformulated for vectorization — not a different, cheaper model. See `docs/checkpoint.md` Phase 3 notes for the full investigation (including the negative-result probe that led here).

**Monte Carlo full-campaign measurement (2026-06-16, `bench_mc_runner_wall_ms` in `bench_monte_carlo.cpp`, same environment):** end-to-end `run_monte_carlo()` — N=1000 independent EKF runs × 1000 steps each, two-body + injected process noise (math.md §6 construction), distributed across the 4-thread pool (`k_mc_threads`). **141.6 ms**, consistent ±1ms across repeated runs — 5.7x inside the 800ms target. For reference, the single-run EKF predict+update cost measured above (~0.26 μs) times 1000 runs × 1000 steps extrapolates to roughly 260 ms of pure filter-update compute if run sequentially on one thread; landing at 142 ms across 4 threads is consistent with real parallelism (not just headroom from a fast single-thread baseline), though this comparison is an estimate, not a controlled 1-thread-vs-4-thread benchmark — `run_monte_carlo()`'s thread count (`k_mc_threads`) is a fixed internal constant, not an exposed parameter, so a true scaling curve wasn't measured this session.
