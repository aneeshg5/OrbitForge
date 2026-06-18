# Architecture

This document describes how OrbitForge is actually built: the thread
model, the data flow from a TLE to a rendered, filtered trajectory, and
the two systems-engineering pieces (the lock-free ring buffer and the
pool allocator) that make a 100 Hz simulation loop possible inside a
browser tab.

## Why a browser app, not a server or native app

The simulation, the three filters, and the sensor models all run as
C++17 compiled to WebAssembly, executing entirely client-side. There is
no backend: no API server, no database, no per-user compute cost. The
physics code is the same code whether you're reading it on GitHub or
running it at orbitforge.dev — nothing is hidden behind an API
boundary.

## Thread model

```
Main thread (JS/TypeScript)              Worker thread (C++17 → WASM)
────────────────────────────             ─────────────────────────────
WebGL2 Earth + orbit renderer             Simulation::step() — fixed-rate loop
Chart.js error/NIS panels                 RK4 true-trajectory propagation
Scenario editor, fault controls           KF / EKF / UKF predict + update
Monte Carlo results UI                    GPS sensor model + fault injection
        │                                          │
        └────────── SharedArrayBuffer ─────────────┘
                  SPSCRingBuffer<StateFrame, 512>
```

The worker owns a `Simulation` instance (`engine/include/wasm_api.hpp`)
that ties together the true-trajectory integrator, all three filters,
the GPS sensor model, and fault injection into one `step(dt)` call. Each
call propagates the true state, runs all three filters, and pushes one
`StateFrame` into the ring buffer. `step()` takes `dt` as an explicit
argument and has no dependency on wall-clock time, which is what makes
it directly unit-testable — `start()`/`pause()` are a thin wrapper that
calls `step()` repeatedly from a background thread at a fixed cadence.

The main thread never blocks on the worker. It drains the ring buffer
at the browser's animation-frame rate and renders whatever is
available, so simulation fidelity (fixed-step RK4 at 100 Hz) and render
rate (variable, typically 60 fps) are fully decoupled.

## Data flow: TLE to rendered estimate

```
User picks a satellite (live CelesTrak feed) or pastes a TLE
  │
  ▼
tle_parser.ts — TLE lines → orbital elements
  │
  ▼
init_scenario(ScenarioCfg)  — WASM call
  │
  ▼  Worker thread:
True trajectory: RK4 + J2 + drag + SRP (the integrator, not SGP4 —
this is intentionally more accurate ground truth than the TLE itself)
  │
  ├── GPS measurement: R_ecef_eci · r_true + N(0, σ²_gps · I)
  │
  ▼
Filter step, all three simultaneously, every tick:
  KF  predict + update  (linearized about a reference orbit — diverges
                          visibly on a curved orbit; this is intentional,
                          see docs/math.md)
  EKF predict + update  (analytical Jacobians, no finite differencing)
  UKF predict + update  (square-root form, sigma points propagated
                          through the same RK4 used for the true state)
  │
  ▼
StateFrame pushed to the ring buffer
  │
  ▼  Main thread:
ring_reader.ts drains frames → WebGL2 renderer (orbit paths, covariance
ellipsoids) + Chart.js panels (position/velocity error, NIS)
```

Filters never see the true trajectory — only the noisy GPS measurements
synthesized from it. The interesting quantity OrbitForge shows that a
pure SGP4 propagator can't is the *estimation error*: how far off, and
how (in)consistent, each filter's belief about its own uncertainty is.

## Memory systems

### Lock-free SPSC ring buffer

`engine/include/memory/ring_buffer.hpp` — `SPSCRingBuffer<T, N>` is the
single channel between the simulation worker (producer) and the
renderer (consumer). No mutex, no condition variable.

- Capacity `N` must be a power of two so index wraparound is a bitmask
  (`w & (N-1)`), not a modulo.
- `write_pos_` and `read_pos_` are each given their own 64-byte-aligned
  cache line, with explicit padding between them. They're written by
  different threads; without the padding they'd share a cache line, and
  every push would invalidate the consumer's cached copy of a line it
  never actually touched (false sharing).
- Memory ordering: `relaxed` load on a thread's own index (no
  cross-thread sync needed), `acquire` load on the other thread's index
  (so the data it wrote becomes visible), `release` store after writing
  data (so the index update is only visible once the data behind it
  is). No sequentially-consistent fences anywhere on the hot path.
- A full buffer drops the new frame rather than blocking the producer —
  the simulation never stalls waiting on the renderer.

In WASM, `std::atomic<size_t>` on a `SharedArrayBuffer`-backed object
compiles to `Atomics.load`/`Atomics.store` with matching ordering
semantics — the lock-free guarantees carry over from native to the
browser unchanged.

### Pool allocator

`engine/include/memory/pool_alloc.hpp` — fixed-size, cache-line-aligned
block pool. Once a scenario is initialized, the simulation tick
allocates nothing on the heap: filter state is stack-allocated
fixed-size Eigen matrices, and any working buffers that need to persist
across ticks come from a pool sized once at scenario init. `malloc` in
a tight 100 Hz loop introduces allocator-lock and fragmentation latency
that's hard to bound; pool allocation is O(1) and has no syscall in the
hot path.

### Monte Carlo: Structure-of-Arrays ensemble

`engine/include/monte_carlo/ensemble.hpp` — for an N-run consistency
campaign, per-run state is stored as one contiguous array per field
(`pos_x[N]`, `pos_y[N]`, ...) rather than an array of per-run structs.
The batch RK4 inner loop iterates over all N runs at a single timestep,
so this layout makes that access pattern cache-sequential and
SIMD-vectorizable instead of striding through `sizeof(RunState)` per
access. Measured improvement over the naive array-of-structs layout is
in `docs/benchmarks.md`.

## Fault injection

Faults are applied at the sensor model layer, not the physics layer:
the true trajectory keeps propagating untouched, and only what the
filter *sees* changes (a GPS measurement spike, a dropout window, an
unmodeled maneuver, a sensor bias). This is the cleanest way to show
the core idea of state estimation — there's a difference between what
is actually happening and what the filter believes is happening, and
that gap is exactly what the filter's covariance is supposed to track.

`set_fault(FaultConfig)` writes into a single-element atomic queue
(`engine/include/faults/fault_injector.hpp`) that the worker thread
checks each tick — no locks between the UI thread setting a fault and
the worker thread picking it up.

## Further reading

- [docs/math.md](math.md) — full derivations: equations of motion, RK4/RK45,
  the analytical EKF Jacobians (including the full J2 Jacobian), the
  UKF square-root sigma-point formulation, and the sensor models.
- [docs/benchmarks.md](benchmarks.md) — measured step times, ring buffer
  throughput, and the AoS-vs-SoA Monte Carlo comparison.
- [docs/checkpoint.md](checkpoint.md) — a running build log: what was
  built, what broke, and how it was verified, in the order it happened.
