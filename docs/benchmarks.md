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

## Measurements

*(Populated after Phase 1 benchmarks run)*
