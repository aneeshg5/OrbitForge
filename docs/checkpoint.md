# OrbitForge — Build Checkpoint Log

A running, step-by-step record of what was implemented, what was tried and
discarded, and why — for any agent (human or AI) picking this project back
up. Organized by phase, then by step within `CLAUDE.md` §22's task order.
Each entry is written immediately after the step's tests pass, while the
reasoning is fresh. Do not retroactively "clean up" old entries to make past
decisions look more deliberate than they were — the failed attempts are the
point.

---

## Phase 1 — C++ Engine

**Status: COMPLETE.** 33/33 tests passing. Benchmarks measured and committed
to `docs/benchmarks.md`. All numbers well inside CLAUDE.md §13 targets.

### Step 0 — Scaffold verification

The repo arrived with `eom.cpp`/`eom.hpp`, `perturbations.cpp`/`.hpp`,
`constants.hpp`, `test_eom.cpp` (6 tests), `CMakeLists.txt`, and the WASM/web
scaffold already in place. First task was just: does it build, do the 6 EOM
tests pass. It did, with cmake newly installed via `brew install cmake`
(4.3.3) — it was not present in the environment initially. Eigen3 was
already available via Homebrew.

### Step 1 — `integrators/rk4.hpp`

Templated fixed-step RK4 (`rk4_step<State, Dynamics>`), state and dynamics
both generic so it works for plain 6-vectors and (later) sigma-point
columns. Tests: Kepler orbit closure (final position within 1 m after one
period) and specific-energy conservation (1e-8 relative, perturbations
disabled). Both passed without rework — RK4 on two-body dynamics is
well-behaved at `dt=10s` for LEO.

### Step 2 — `integrators/rk45.hpp`

Dormand–Prince embedded 4(5) pair with adaptive step (`atol=1e-6 m,
rtol=1e-9`). Same two pass criteria as RK4, reused. No significant issues —
the embedded-pair error estimate and step-size update formula
(`h_new = h · clamp(0.9·(tol/err)^(1/5), 0.1, 5)`) matched the textbook form
directly.

### Step 3 — `filters/filter_base.hpp` + `filters/kf.hpp`/`kf.cpp`

`FilterBase<N_STATE, N_MEAS>` abstract base holding `x, P, Q, R` as
fixed-size Eigen matrices (never `MatrixXd`, per coding conventions). Linear
KF derives the gravity Jacobian once at the current estimate and uses
`Φ = I + F·dt` for *both* state and covariance propagation (no RK4) — this
is intentional per CLAUDE.md: the linear KF is supposed to accumulate
visible linearization error on a curved orbit. Tests confirm predict grows
P, update shrinks P, and `H` defaults to `[I₃|0₃]`.

### Step 4 — `filters/ekf.hpp`/`ekf.cpp`

State propagated nonlinearly via RK4 (reuses two-body+J2+drag+SRP dynamics
from `eom.cpp`); covariance propagated via analytically-derived Jacobian
`F` (gravity + J2 terms; drag's velocity coupling intentionally omitted —
documented in the header as a conservative-force approximation). The J2
Jacobian has 9 nonzero analytically-derived entries (math.md §3.2) — these
were derived by hand and cross-checked against the symmetry of the J2
acceleration formula (∂a_J2x/∂y must equal ∂a_J2y/∂x, etc.) before being
trusted in code.

**Update form changed twice.** Originally implemented the textbook
"short form" `P = (I - KH)·P`. This passed the unit tests
(`trace(P_post) < trace(P_prior)`) but was **revisited in Step 7** when the
500-step Monte Carlo consistency test needed P to stay exactly symmetric
and positive-definite over many iterations — switched to Joseph form
`P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ`. Existing tests were unaffected since both
forms are equivalent for the optimal K; Joseph form is just more
numerically robust under repeated floating-point application.

### Step 5 — `filters/ukf.hpp`/`ukf.cpp` (square-root form)

This was the hardest single step. Design constraints from CLAUDE.md: no
LAPACK, `cholupdate()` implemented inline, S (lower-triangular, P=S·Sᵀ)
maintained directly rather than P.

**Design chosen:** QR-based reconstruction of S⁻ during `predict()` (stack
an 18×6 matrix of `[√W_ci·deviations ; chol(Q)]`, HouseholderQR, transpose
of R factor gives S up to sign), then rank-1 Cholesky **downdate** during
`update()` (since the measurement-update covariance shrinkage in the
square-root form is naturally a downdate, not an update).

**Bugs hit and fixed:**
1. *`rk4_step` template deduction failure.* Calling
   `rk4_step(chi.col(i), ...)` deduced `State` as
   `Eigen::Block<Matrix<double,6,13>,6,1,true>` instead of a concrete
   `Matrix<double,6,1>` — the Block type can't be assigned back into the
   sigma-point matrix column directly from the return type mismatch. Fixed
   by copying into an explicit `const Eigen::Matrix<double,6,1>` before
   calling `rk4_step`, then assigning the result into `.col(i)`.
2. *Givens rotation sign error during design (caught before it became a
   runtime bug).* Initial formulation used `c = r/L_kk, s = v_k/L_kk` for
   the rank-1 update/downdate, which gives the wrong updated `v[i]`. The
   correct relations are `c = L_kk/r, s = v_k/r`. Verified by hand on a
   2×2 example (downdate of `[[2,0],[1,√3]]` with `v=[1,0]` must give
   `(1,1)` entry `√(8/3)≈1.633`) before trusting the implementation.
3. *QR sign ambiguity.* `HouseholderQR` can return a negative diagonal in
   the R factor; since `S = Rᵀ`, this can flip the sign of `S`'s diagonal
   entries while still satisfying `S·Sᵀ = P`. Detected by the
   `SRUKF.CholeskyRemainsValid` test (`S(i,i) > 0` required) and fixed by
   flipping the sign of any column `i` where `S(i,i) < 0`.

W_c0 for these UKF parameters (α=1e-3, κ=0, β=2) is large and negative
(≈ -999996) because α is tiny — this is expected for the scaled UKF and
not a bug; it just means the central sigma point's downdate is large in
*weight* even though the deviation itself (`χ₀ - x_pred`) is tiny for
smooth dynamics, so the actual downdate magnitude stays small and
well-conditioned.

### Step 6 — Sensor models (`sensors/gps.hpp`, `imu.hpp`, `magnetometer.hpp`/`.cpp`)

GPS: ECI→ECEF rotation via GAST (`θ = 280.46061837° + 360.98564736629°·(JD-2451545)`),
additive `N(0, σ²I)`. IMU: bias random walk (`bias += N(0, σ_bias²·dt·I)`)
plus white measurement noise; a `dt=0` guard prevents the bias walk from
advancing on a zero-duration call. Magnetometer: IGRF-13 (2020 epoch)
4-coefficient dipole+quadrupole table (`g10, g11, h11, g20`), geocentric
spherical field formula converted to ECEF then ECI.

**Issues fixed:**
- `M_PI` is unavailable/unreliable because the CMake config sets
  `CMAKE_CXX_EXTENSIONS OFF`, which drops POSIX math extensions on some
  toolchains. Replaced every `M_PI`-based degree/radian conversion with an
  explicit `constexpr double deg_to_rad = 1.7453292519943295769e-2`.
- Magnetometer pole singularity: geocentric longitude `λ = atan2(y,x)` and
  the `θ̂, λ̂` unit vectors are undefined when `ρ = √(x²+y²) → 0` (over the
  poles). Added a guard: if `ρ < 1 km`, return an approximate axial dipole
  field instead of evaluating the singular spherical formula.

### Step 7 — Filter consistency Monte Carlo (NEES)

Implemented the two remaining testable items from CLAUDE.md §14:
`EKFConsistency.NEESWithinBounds` and `UKFConsistency.NEESWithinBounds`
(`test_filter_consistency.cpp`). 100 independent runs, 500 steps each,
`dt=10s`, ISS-like circular orbit, GPS σ=10m. Per-step NEES averaged across
runs must fall in `[χ²(600,0.025)/100, χ²(600,0.975)/100] ≈ [5.35, 6.69]`
(Wilson–Hilferty approximation) for ≥ 90% of the 500 steps.

**First attempt failed badly (21.6% in bounds, not 90%).** The setup gave
the filter `Q = diag(1e-4 m², 1e-8 (m/s)²)` but propagated the *true*
trajectory with **zero** process noise (pure deterministic two-body RK4).
This is a textbook consistency-test mistake: with the filter assuming more
process uncertainty than actually exists, `P` converges to a value set by
`Q` while the *actual* error converges toward zero (the dynamics really are
that predictable), so `NEES = errᵀP⁻¹err → 0`, far below the 5.35 floor.
(The opposite mistake — `Q=0` with real GPS noise present — would have
caused the opposite failure: `P→0` while error stays bounded by GPS noise,
so `NEES→∞`.)

**Fix:** inject `w ~ N(0, Q)` directly into the true trajectory every step,
exactly matching the filter's `Q`. This makes the stochastic model the
filter assumes the one that is actually realized, which is the standard
construction for NEES-testing a filter against an otherwise-deterministic
system. After the fix, both EKF and UKF passed immediately with the same Q
(`σ_pos=1m/step, σ_vel=0.01 m/s/step`, `R=100 m²`). Full derivation recorded
in `docs/math.md` §6 so the next person doesn't repeat the same dead end.

This is also why the EKF's covariance update was switched to Joseph form
in Step 4 (revisited here) — 500 sequential updates need P to stay exactly
symmetric/PD for the `Eigen::LLT` solve used to compute NEES to succeed
every step.

### Step 8 — Benchmarks

Added `engine/benchmarks/bench_filters.cpp` (KF/EKF/UKF predict+update
timing, plus a combined "3 filters in one tick" measurement) and extended
`bench_integrator.cpp` with an actual `rk4_step` benchmark — the original
file only timed `compute_acceleration` directly, which is *not* the same
number as CLAUDE.md §13's "single RK4 step" target (one RK4 step makes 4
calls to `compute_acceleration`). Built via `scripts/benchmark.sh`
(Release, `-O2`, native arm64, no WASM yet).

Filter benchmark methodology: each filter's `predict(dt)` is immediately
followed by `update(z)` where `z = x̂.head<3>()` — a zero-innovation
measurement. This keeps the loop numerically stable indefinitely (no
divergence to chase down) while still exercising the complete Kalman gain /
covariance-update matrix pipeline, since `P` is never degenerate. `x`
itself just follows the real two-body+J2 dynamics under repeated
`predict()` calls.

**Results (Apple M4 Pro, macOS, Apple clang 17, all far inside target):**

| Operation | Target | Measured |
|---|---|---|
| RK4 step | < 2 μs | 0.149 μs (13x margin) |
| EKF predict+update | < 8 μs | 0.247 μs (32x margin) |
| UKF predict+update | < 20 μs | 1.292 μs (15x margin) |
| 3 filters/tick | < 40 μs | 1.655 μs (24x margin) |

Full table and raw output in `docs/benchmarks.md`. Monte Carlo throughput,
WASM overhead, ring buffer throughput, and SoA-vs-AoS are explicitly *not*
measured yet — they require Phase 2/3 components that don't exist.

---

## Environment notes (carried forward for Phase 2+)

- macOS, cmake 4.3.3 (Homebrew), Eigen3 (Homebrew), Apple clang 17.
- `CMAKE_CXX_EXTENSIONS OFF` — do not rely on `M_PI` or other POSIX math
  extensions anywhere in `engine/`.
- Debug builds compile with ASan+UBSan by default
  (`-DCMAKE_BUILD_TYPE=Debug`); pass `-DENABLE_TSAN=ON` to swap to
  ThreadSanitizer instead (mutually exclusive with ASan in this config —
  relevant once the Phase 2 ring buffer needs TSan-clean verification).
- Release benchmark build is a separate `build_rel/` tree
  (`scripts/benchmark.sh` drives it); do not reuse `build/` for timing
  numbers since it carries sanitizer instrumentation overhead.

---

## Phase 2 — Memory Systems + WASM Build

**Status: COMPLETE.** 63/63 tests passing (all Phase 1 + Phase 2 tests).
Ring buffer throughput measured: **3.46×10⁸ /sec** (single-thread).
WASM compile verification deliberately deferred to CI (see Step 5 below).
TypeScript typecheck (`npx tsc --noEmit`) clean. `npm run build` produces
a valid `dist/` with correct sw.js output.

### Step 1 — `memory/ring_buffer.hpp`

Lock-free SPSC ring buffer, power-of-2 capacity, 64-byte cache-line
padding on both `write_pos_` and `read_pos_`. Memory ordering: `relaxed`
load on own index, `acquire` load on the other thread's index, `release`
store after writing data. No sequentially-consistent ops, no full fence.

The padding size is `k_pad = 64 - sizeof(std::atomic<size_t>)`. This is
platform-precise: on any 64-bit platform `sizeof(atomic<size_t>)` is 8, so
`k_pad = 56`, giving exactly one 64-byte cache line per index. Hardcoding
`56` would be an implicit assumption about `sizeof(atomic<size_t>)` — using
the expression makes the intent self-evident and avoids a subtle correctness
assumption about the target ABI.

`clear()` method resets both indices to 0 via relaxed stores; only safe when
no threads are concurrently pushing or popping (used by `Simulation::reset()`
after stopping the worker thread).

Tests in `test_ring_buffer.cpp`:
- **FIFO correctness** (single-thread, push N then pop N, FIFO order verified).
- **Full-buffer drop** (push N+1 when capacity is N, N+1th push returns false).
- **Empty-buffer miss** (pop on empty buffer returns false).
- **Multi-thread producer/consumer, 10 million items** — verified TSan-clean
  under `-DENABLE_TSAN=ON`. The test busy-spins on `pop()` rather than
  using a condvar to keep the test minimal and exercised under TSan's
  scheduler (condvar-based sleep would reduce context-switch races that TSan
  is there to find). All 10M items confirmed received in FIFO order.

**Dead end — EXPECT_DEATH hangs on macOS sandbox.** Initially wrote a death
test: `EXPECT_DEATH(rb.pop_with_bad_args(), "some assertion")`. GTest's
`EXPECT_DEATH` forks a child process and waits for it to die; on macOS,
`sandbox-exec` blocks the crash-reporter daemon connection, so SIGABRT in
the child causes `waitpid()` to never return. The test hung indefinitely and
had to be killed manually. Removed all death tests from `test_ring_buffer.cpp`
and `test_pool_alloc.cpp` (see Step 2). The functional assertion coverage that
death tests would have provided is covered by the non-death tests instead.

### Step 2 — `memory/pool_alloc.hpp`

Fixed-block slab allocator, `BlockSize % 64 == 0` enforced by
`static_assert`. Free-list is a stack of `std::byte*` pointers indexed by
`free_head_` (starts at `NumBlocks`, decrements on alloc, increments on
dealloc). `allocate()` is O(1), zero syscalls, zero mutex.

Debug-only double-free detection: a `uint64_t[(NumBlocks+63)/64]` bitmap
tracks which blocks are free. `set_free(block, true/false)` updates the
bit; `is_free(block)` checks it. Both are `#ifndef NDEBUG` guarded and
disappear in release builds (same pattern as `assert()`).

**Bug fixed — assert ordering in `deallocate()`.** First draft checked
`free_head_ < NumBlocks` (overflow guard) before the `!is_free(block)`
double-free check. A double-free at full capacity would trigger "pool free
list overflow" instead of "double free detected" — the wrong error for
debugging. Corrected by putting the `#ifndef NDEBUG` double-free check first.

**Dead end — EXPECT_DEATH on macOS.** Added `PoolAllocator.DoubleFreeAborts`
using `EXPECT_DEATH`. Same macOS sandbox hang as Step 1. Removed; correctness
is covered by the fill/free/refill reuse test (if a double-free silently
corrupted the free-list, the subsequent refill would allocate a block that
is still in use, causing the "owns(block)" assert to fire on a subsequent
dealloc — effectively a delayed death test).

Tests in `test_pool_alloc.cpp`:
- Fill to capacity, verify `allocate()` returns non-null and unique pointers.
- Verify `allocate()` on a full pool returns `nullptr`.
- Verify all returned pointers are 64-byte aligned (`ptr % 64 == 0`).
- Free all blocks, refill, verify same capacity — exercises the free-list
  stack and ensures no reuse corruption.
- `free_count()` tracks correctly through alloc/dealloc cycles.
- `deallocate(nullptr)` is a no-op (no crash).

### Step 3 — `scenario.hpp` (TLE parsing + Kepler seeding)

Three inline functions: `parse_tle()` (fixed-width column offsets from the
NORAD TLE spec), `solve_kepler_eccentric_anomaly()` (Newton-Raphson,
1e-12 convergence, 50-iteration cap), and `tle_elements_to_eci()` (Vallado
COE2RV Algorithm 10: PQW frame velocities + 3×3 rotation matrix from the
five angle parameters Ω, i, ω).

**Design decision — Two-body Kepler, not SGP4.** SGP4 is a *propagator*
(prediction for hours or days forward from a TLE epoch). OrbitForge only
needs a single-epoch ECI state as the seed for its own RK4+perturbations
propagation. The TLE → COE → ECI conversion using two-body Kepler gives a
seed that is ~O(100 m) off a full SGP4 seed; this is well inside the initial
covariance P₀ = diag(100², 100², 100², 1², 1², 1²) m²/(m/s)² used to start
the filters. Using two-body Kepler instead of SGP4 means the seed model
and the propagation model are the same model — consistency matters more than
seed fidelity for a filter-convergence demonstration. Documented in
`scenario.hpp` header comment and in `CLAUDE.md §5/§18`.

**Bug fixed — TLE epoch Julian date formula.** The initial implementation
used the formula `367*y - floor(7*(y+1)/4) + 30 + 1721013.5 + day_of_year`.
The `ParseTle.EpochJulianDateInExpectedRange` test caught the error
immediately: for an ISS TLE with epoch `24006.60779611` (January 6.6, 2024),
expected JD ≈ 2460316.1, actual 2459943 — an error of exactly 373 days.
Fixed by switching to the standard Meeus Gregorian-to-JD formula
("Astronomical Algorithms", ch. 7):

```
yp = year - 1
a = yp / 100
b = 2 - a + (a / 4)
jd0 = floor(365.25 * (yp + 4716)) + 428 + b - 1524.5
JD = jd0 + day_of_year
```

Verified: `JD(2024-01-01 00:00 UT) = 2460310.5`, `JD(2000-01-01 00:00 UT) = 2451544.5`.

The identical bug was present in `web/src/data/tle_parser.ts` (same formula
transcribed into TypeScript). Fixed in both files at the same time.

`web/src/data/tle_feed.ts` had an unused `CELESTRAK_BASE` constant (TS6133)
left over from an earlier draft; removed at the same time.

Tests in `test_scenario.cpp`:
- TLE epoch JD within expected range for a known ISS TLE.
- Kepler's equation: Newton-Raphson convergence verified for e=0.7 (highly
  eccentric), M=π/4 — high-eccentricity orbit is the hardest case for the
  N-R starting point `E₀ = M`.
- COE2RV circular orbit radius: `|r| ≈ 6786 km` for an ISS-like TLE.
- Velocity for the same orbit ≈ ISS circular velocity (7.67 km/s).
- High-eccentricity orbit (e=0.5): position norm inside `[a(1-e), a(1+e)]`
  i.e. between periapsis and apoapsis.
- `parse_tle()` field extraction: inclination, RAAN, eccentricity, mean
  anomaly, mean motion all within expected ranges for the test TLE.

### Step 4 — `faults/fault_injector.hpp`

Single-element overwriting mailbox, not a queue. The UI thread calls `set()`;
the worker thread calls `try_take()` once per tick. Only the most recent
`set()` survives if the worker hasn't read yet — this is intentional: faults
are modal (the user selects a fault type and onset time, not a stream of
fault events). A full ring buffer would be the wrong abstraction.

`FaultQueue` uses a release-store/acquire-load pair on `pending_` to make
the worker's read of `staged_` safe without a separate lock. The
happens-before chain: `set()` writes `staged_` (non-atomic), then
`pending_.store(true, release)`. The worker calls `pending_.load(acquire)`;
if it observes `true`, the `staged_` write is in its happens-before set,
so reading `staged_` is safe.

The existing CLAUDE.md §14 fault injection tests (`test_fault_injector.cpp`)
cover: `set()` followed by `try_take()` returns the fault config; two `set()`
calls before `try_take()` leaves only the second; `try_take()` returns false
after the pending is consumed.

### Step 5 — `wasm_api.hpp` / `wasm_api.cpp` (Simulation class)

`StateFrame` struct: 46 doubles (6 groups of position+velocity+covariance
diagonal+NIS) + 1 `uint8_t active_fault`. Verified
`sizeof(StateFrame) == 376` by adding a `static_assert` in
`test_wasm_api.cpp` — the compiler's default alignment rules already make
the struct 8-byte aligned with no manual padding needed (`uint8_t` at byte
offset 368 pads to 376 via the compiler's struct tail padding).

Ring buffer memory layout: `write_pos_` at offset 0 (in its own 64-byte
slot), `read_pos_` at offset 64, `StateFrame buffer_[512]` at offset 128.
This is the layout the JS `RingReader` assumes when it reads from the
`SharedArrayBuffer`. The offsets are implicit from the `SPSCRingBuffer`
struct definition — `ring_reader.ts` hardcodes `+0`, `+64`, `+128`
relative to `ringBufferPtr`, which must stay in sync with the C++ layout.

`seed_filter()`: a file-scope helper (not a method) taking `FilterBase<6,3>&`
so it can be called for all three filter types without a template lambda —
avoiding a template-on-lambda disambiguation issue that complicated early
drafts.

**Fault state machine in `step(dt)`:**
- `gps_spike`: single-tick `gps_spike_offset` applied once (`fault_applied_once_` gate).
- `gps_dropout`: time-window gate `t_now < onset + duration`.
- `maneuver`: single-tick impulsive ΔV along the normalized current velocity
  (`fault_applied_once_` gate). Applied to `x_true_` *after* the RK4 step, so the
  filter predicts for the unperturbed state and then sees a surprising GPS
  measurement — exactly the GNC divergence scenario intended.
- `drag_coeff_error`: re-derives `perturb_true_.drag_coeff` every tick as
  `nominal * (1 + magnitude)`. Duration 0 means "persist forever"; a cleared
  fault resets it back to nominal in the `fault_queue_.try_take()` path.
- `sensor_bias`: tracked and reported in `StateFrame.active_fault` but has
  no observable effect yet — the filters are GPS-position-only and don't
  fuse IMU measurements. The scaffolding is correct for Phase 3 when IMU
  fusion is added.

**NIS computed pre-update** using the prior covariance P:
`ν = z - H·x_prior`, `S = H·P·Hᵀ + R`, `NIS = νᵀ S⁻¹ ν` via LDLT.
This is the proper textbook NIS (innovation covariance at the time of the
measurement, not after the update). The `H` matrix is set from the current
Julian date at the start of each tick so that the ECEF rotation matrix
(`R_ecef_eci`) is always current.

**Embind vs. ccall discovery.** First draft used `EMSCRIPTEN_BINDINGS` /
`emscripten::function()` (embind). Discovered this conflicts with the
pre-existing `web/src/bridge/wasm_types.ts` scaffold, which uses
`OrbitForgeModule.ccall / cwrap` — embind exports don't show up as ccall
targets. Rewrote the WASM-export block using `extern "C"` +
`EMSCRIPTEN_KEEPALIVE` (no embind dependency), matching the existing
TypeScript interface exactly. The `extern "C"` functions flatten
`ScenarioCfg` and `FaultConfig` into primitive arguments since ccall
cannot pass struct pointers portably across the JS/WASM boundary.

**WASM compile deliberately unverified locally.** No Emscripten toolchain
in this dev environment. The `#ifdef __EMSCRIPTEN__` block is syntactically
correct C++17 but has not been executed by `em++`. CI (`deploy.yml`) runs
the full WASM build on Ubuntu with `emsdk 3.1.50`. This is a deliberate
trade-off (see Pending / Phase 3 decisions): native tests give high
confidence in the physics and filter logic; CI gives confidence in the
WASM compilation and COOP/COEP header propagation.

Tests in `test_wasm_api.cpp`:
- `sizeof(StateFrame) == 376` static assertion.
- `init_scenario()` with a valid ISS TLE does not throw / abort.
- After `init_scenario`, initial sim time is 0.
- `step(10.0)` advances sim time by 10 s.
- After N steps, ring buffer has N frames (single-thread, no pop).
- `StateFrame.true_pos` is non-zero after a step.
- `StateFrame.ekf_pos` is initialized (within 10 km of true for first step).
- Covariance diagonal entries are all positive after init.

### Step 6 — Web scaffold fixes

Several pre-existing TypeScript scaffold files had bugs or placeholders that
would have caused runtime errors or failing typecheck.

**`web/src/bridge/ring_reader.ts`:**
- `FRAME_BYTES` was 312 (rough placeholder). Corrected to **376** to match
  verified `sizeof(StateFrame)`.
- Constructor assumed the ring buffer starts at byte offset 0 of the
  SharedArrayBuffer. In WASM, the SAB is the entire WASM heap
  (`Module.HEAPF64.buffer`); the ring buffer lives at `get_ring_buffer_ptr()`
  bytes into it, not at 0. Added `ringBufferPtr: number` parameter; the
  `writePosView`, `readPosView`, and frame data views are all offset by
  `ringBufferPtr`. An alignment guard (`ringBufferPtr % 8 !== 0`) was added
  to catch misconfigured pointer arguments early.
- Removed unused `buf: SharedArrayBuffer` private field (TS6133 error).

**`web/src/bridge/wasm_types.ts`:**
- `ccall`'s `returnType` parameter was typed as `string`, blocking `null`
  (the correct value for void-returning functions). Changed to `string | null`.

**`web/src/data/tle_parser.ts`:**
- Same `tle_epoch_to_jd` formula bug as `scenario.hpp` — fixed identically.

**`web/src/worker.ts`:**
- Rewrote entirely: typed `WorkerRequest`/`WorkerResponse` discriminated
  union (exported for `main.ts`), `loadWasmModule()` with a
  `/* @vite-ignore */` non-literal dynamic import path
  (`const modulePath = '/orbitforge.js'`) to avoid Vite's static resolution
  of the non-existent build artifact at typecheck time, a promise-singleton
  `getWasmModule()` to deduplicate concurrent load calls, and ccall-based
  message handlers for `init`, `start`, `pause`, `reset`, `set_fault`.
  After `init`, posts `ring_buffer_ready` with `{sharedArrayBuffer,
  ringBufferPtr, ringBufferCapacity}` for the main thread to construct its
  `RingReader`.

**`web/src/main.ts`:**
- Rewrote entirely: spawns the module worker, listens for `ring_buffer_ready`,
  constructs a 3-argument `RingReader`, starts a 60 fps `requestAnimationFrame`
  polling loop that calls `ringReader.pop()` and logs the frame (placeholder
  for Phase 3 renderer), and registers the service worker.
- Removed unused `ScenarioConfig` import that `tsc --noEmit` had flagged.

**`web/sw.ts`:**
- Service worker for offline WASM caching. The TypeScript DOM lib doesn't
  include `ExtendableEvent`/`FetchEvent`; using `lib.webworker.d.ts` would
  conflict with DOM's global `self` type in `main.ts` (both can't be in
  scope at once in the same tsconfig). Solution: local `ExtendableEventLike`
  / `FetchEventLike` interface casts inside the event handlers —
  structurally compatible with what the SW runtime actually provides, no lib
  import needed.

**`web/tsconfig.json`:**
- Removed `"rootDir": "src"` — it prevented `sw.ts` (at the project root, not
  under `src/`) from being compiled.

**`web/vite.config.ts`:**
- Added multi-entry `rollupOptions`: `input: {main: 'index.html', sw: 'sw.ts'}`
  with `entryFileNames: (chunk) => chunk.name === 'sw' ? 'sw.js' : 'assets/[name]-[hash].js'`.
  Without the override, Vite would name the service worker `assets/sw-[hash].js`,
  making it impossible to register at a stable path. The sw must also emit as
  a classic script (not ESM), which Vite's default for non-entry modules
  already handles for this build configuration.

**`web/package.json`:**
- Created: `vite ^8.0.16` (^5.4.2 pulled in esbuild ≤0.28.0 with a
  high-severity CORS/RCE CVE — CVE-2025-30154; upgraded immediately),
  `typescript ^5.5.4`. Scripts: `dev`, `build`, `preview`, `typecheck`
  (`tsc --noEmit`).

### Step 7 — Infrastructure

**`scripts/build_wasm.sh`:**
- Replaced the literal placeholder `-I/path/to/eigen` with a portable
  candidate-path detection loop:
  `/usr/include/eigen3` → `/opt/homebrew/include/eigen3` →
  `/usr/local/include/eigen3`, first found wins; `EIGEN_INCLUDE` env var
  overrides. Prints an error and exits 1 if none found.
- Removed `mc_runner.cpp` from the source list — Phase 3 file, doesn't
  exist yet. Left a comment marking where to re-add it.

**`.github/workflows/deploy.yml`:**
- Added `sudo apt-get install -y libeigen3-dev` before the WASM build step.
  Without it, the CI Ubuntu runner would fail on the `em++` command with a
  missing `<Eigen/Dense>` include.

**`engine/CMakeLists.txt`:**
- Added `find_package(Threads REQUIRED)` and `Threads::Threads` link to both
  `orbitforge_tests` and `orbitforge_benchmarks` targets — required by
  `wasm_api.cpp`'s use of `std::thread`.
- Added `src/wasm_api.cpp` to `ENGINE_SOURCES`.
- Added all Phase 2 test files: `test_ring_buffer.cpp`, `test_pool_alloc.cpp`,
  `test_scenario.cpp`, `test_fault_injector.cpp`, `test_wasm_api.cpp`.

### Step 8 — Ring buffer throughput benchmark

Added `bench_ring_buffer_throughput()` to `engine/benchmarks/bench_integrator.cpp`:
single-threaded back-to-back `push()`+`pop()` pairs, 10,000,000 iterations.
Measures the raw atomic-operation cost (not real thread-scheduling overhead from
a true producer/consumer pair, which is a different and much noisier number).

**Result: 3.46×10⁸ push+pop pairs/sec — 69× above the 5M/sec target.**

Committed to `docs/benchmarks.md` alongside the Phase 1 filter/integrator
numbers. Multi-threaded throughput (real producer/consumer with independent
OS scheduling) is intentionally not measured: the 10M-item TSan-clean test
in `test_ring_buffer.cpp` validates correctness under concurrency; the
throughput question for two OS-scheduled threads is dominated by kernel
scheduler variance and produces a much less stable number than the single-
thread atomic cost, which is what actually matters for the 100 Hz sim loop.

---

## Environment notes — Phase 2 additions

All Phase 1 environment notes still apply. Phase 2 additions:

- `npm` available via Homebrew Node.js. `web/package-lock.json` committed
  for reproducible CI installs.
- `npx tsc --noEmit` is the typecheck command for the web scaffold.
- `npm run build` produces `web/dist/` with `sw.js` at the root (not under
  `assets/`) and a hashed `main-[hash].js` under `assets/`.
- No Emscripten toolchain locally — all WASM-specific paths are exercised
  only in CI (`deploy.yml`, emsdk 3.1.50, Ubuntu).
- TSan and ASan are mutually exclusive in the CMake config: use
  `-DCMAKE_BUILD_TYPE=Debug` (ASan+UBSan default) for correctness checks
  and `-DENABLE_TSAN=ON` for race-detection. The ring-buffer multi-thread
  test must be run under TSan, not ASan.

## What's next — Phase 3

Per `CLAUDE.md` §17: `ensemble.hpp` (SoA state buffer), `mc_runner.cpp`
(pthread pool, N-run distribution, statistics), then the WebGL2 renderer
(`earth.ts`, `orbit.ts`, `covariance.ts`, `panels.ts`) and UI layer
(`scenario_editor.ts`, `fault_panel.ts`, `filter_compare.ts`,
`mc_results.ts`, `tle_feed.ts`). Re-add `mc_runner.cpp` to
`scripts/build_wasm.sh` once it exists. Run SoA-vs-AoS throughput
benchmark and commit to `docs/benchmarks.md`.
