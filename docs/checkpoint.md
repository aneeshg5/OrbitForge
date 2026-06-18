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

---

## Phase 3 — Monte Carlo + Visualization (in progress)

### Step 1 — `monte_carlo/ensemble.hpp` (SoA state buffer)

`EnsembleState<MaxN>` holds 6 parallel `std::array<double, MaxN>` (pos_x/y/z,
vel_x/y/z), `MaxN` a compile-time template parameter (`k_mc_max_runs = 5000`,
matching CLAUDE.md §11's `mcRuns` upper bound). `set()`/`get()` round-trip
to/from a fixed-size `Eigen::Matrix<double,6,1>` for interop with the
existing filter/dynamics code. `step_ensemble()` propagates runs `[0,n)` by
one RK4 step, reusing the existing `rk4_step()` + `compute_acceleration()`
per run — this is the direct, "obviously correct" implementation, validated
against the single-run path and an 8-run one-period Kepler-closure check in
`test_ensemble.cpp`.

**Dead end — SoA storage alone gave no measurable benefit, and the first
benchmark draft would have reported a fabricated number to match CLAUDE.md's
expectation.** CLAUDE.md §7 claims "~40% throughput on batch RK4 for N=1000
runs" comparing an AoS struct (with an unused 36-double covariance block
dragging position/velocity fields apart in memory) against the SoA layout.
The first benchmark draft measured this directly: `step_ensemble()` (SoA,
processed one run at a time through the same scalar Eigen path as the AoS
version) against an AoS struct processed identically. Result, repeated
across multiple runs: **+0% to +13%, indistinguishable from measurement
noise** — nowhere near 40%.

Investigated why before writing anything down: profiled both layouts at
N=1000 and N=5000, with and without J2 enabled. The result didn't change
with N, which ruled out "dataset too small to exceed cache" as the
explanation by itself. The actual cause: `compute_acceleration()`'s inner
math (sqrt + divides for gravity and J2) is floating-point-latency-bound,
not memory-bandwidth-bound, and **identical in both layouts** — both
versions copy into the same `Eigen::Matrix<double,6,1>` and call the same
scalar function per run. Memory layout cannot speed up a loop whose
bottleneck is FPU latency inside a per-iteration function call, regardless
of how the inputs are stored. CLAUDE.md §7's own text says the real lever is
that SoA storage is "cache-prefetchable, **SIMD-vectorizable**" — the
existing `step_ensemble()` realizes the first half of that sentence and
none of the second half.

**Fix:** wrote `accel_gravity_j2_batch()` + `step_ensemble_fast()` —a batched
gravity+J2 kernel operating directly on the six `double[N]` arrays with no
per-run `Eigen::Matrix` construction and no branch inside the hot loop (the
`enable_j2` toggle selects which of two loop bodies runs, rather than
branching per-element). This is the standard split-RK4 form for a
second-order system `ẍ = a(x)` (math worked out by hand in the header
comment: `a0=a(p); p1=p+h/2·v; v1=v+h/2·a0; a1=a(p1); ...`), proven
algebraically identical to applying the generic `rk4_step()` to
`f(p,v)=(v,a(p))`, and then verified numerically against the existing
per-run path to `1e-9` relative tolerance in
`Ensemble.FastPathMatchesGenericPathWithJ2`/`...TwoBodyOnly`.

Scoped to gravity+J2 only (not drag/SRP) — drag needs a branchy 7-band
atmosphere-altitude lookup and SRP needs a shared (not per-run) sun-direction
vector; neither is a clean fit for this treatment, and J2 is already the
dominant LEO perturbation and the same scope as the EKF/UKF analytical
Jacobians. `step_ensemble()` (the generic path) remains the correct
fallback whenever drag or SRP are enabled — it is not being deleted or
deprecated, just no longer the only path.

**Re-benchmarked with three configurations instead of two** (AoS-scalar,
SoA-scalar, SoA-batched) to make the actual source of any improvement
legible rather than collapsing it into one AoS-vs-SoA number. Results
(N=1000, 100 steps, Release build, repeated 3+ times):
- AoS scalar vs. SoA scalar (`step_ensemble`): **+10–20%**, confirming the
  "layout alone doesn't matter here" finding above.
- AoS scalar vs. SoA batched (`step_ensemble_fast`): **+430–485%**,
  consistent across repeated runs.

This is the honest number, not the CLAUDE.md-estimated one — it is larger,
not smaller, which is a fine outcome, but the actual measured number is
what got committed to `docs/benchmarks.md`, with the methodology and the
three-configuration breakdown explained so a future reader can see why the
two SoA numbers differ so much from each other. Both `EnsembleState` and
`EnsembleWorkspace` (the batched kernel's scratch buffers) are intended to
be heap-allocated once via `std::make_unique` at Monte Carlo run start, not
inside the per-tick hot loop — same allocation discipline as the rest of
the engine.

Tests in `test_ensemble.cpp` (6 tests): set/get round-trip; single run via
`step_ensemble` matches a direct `rk4_step()` call; two runs propagate
independently (no cross-lane contamination); 8-run one-period Kepler
closure (<1 m drift, same criterion as `test_rk4.cpp`); `step_ensemble_fast`
matches `step_ensemble` with J2 enabled; same with J2 disabled (two-body
only).

### Step 2 — `monte_carlo/mc_runner.hpp`/`.cpp` (thread pool + statistics)

`MCConfig`/`MCStats` + `run_monte_carlo(cfg)`: distributes `cfg.n_runs`
independent realizations across a fixed `k_mc_threads = 4` `std::thread`
pool as contiguous slices (`base = n/4`, remainder runs added to the first
`n%4` threads) — matching CLAUDE.md §8's "no work-stealing, runs are equal
cost." Each thread accumulates its own `PartialSums` (squared position/
velocity error, NEES sum/count, NIS sum/count, one entry per timestep) with
zero shared mutable state during the parallel phase — no locks needed
inside `run_one()`, only a single merge pass across the 4 `PartialSums`
after all threads join.

Each run propagates a true trajectory via two-body RK4 with
`w ~ N(0,Q)` injected every step (the same construction Phase 1's
`test_filter_consistency.cpp` validated, documented in `math.md §6`) against
a direct noisy ECI position measurement (not the ECEF-rotated GPS model
`wasm_api.cpp`'s `Simulation` class uses for the live single-scenario path —
documented as a deliberate scope decision in `mc_runner.hpp`'s header
comment, since the two don't need to match for a statistical consistency
campaign and using the simpler model kept this component testable directly
against the already-validated Phase 1 result). `FilterKind` selects KF/EKF/
UKF; a `configure_dynamics()` overload set (mirroring `wasm_api.cpp`'s
`seed_filter()` pattern from Phase 2) sets `perturb_cfg`/`julian_date` for
EKF/UKF and is a no-op for KF, which has neither field.

**Added a general `chi_squared_quantile(p, dof)` rather than reusing the
hardcoded `[5.35, 6.69]` constants from `test_filter_consistency.cpp`.**
CLAUDE.md §8 needs 95% NEES/NIS bounds for arbitrary N (mcRuns ranges
100–5000), not just N=100. Implemented Wilson-Hilferty
(`math.md §6`: `χ²(ν,p) ≈ ν(1 - 2/(9ν) + z_p√(2/(9ν)))³`) with the normal
quantile `z_p` computed by **bisection on the exact CDF**
(`0.5·erfc(-z/√2)`, both available in `<cmath>`) rather than a transcribed
rational-approximation formula for the quantile itself — avoids any risk of
a mistyped coefficient, and the extra iterations cost nothing since this
runs once per campaign summary, not per-tick. Verified by hand before
trusting it: at ν=600, `z_{0.025}=-1.959964` and `z_{0.975}=1.959964` give
`χ²≈534.1` and `≈669.8`, i.e. `/100 → [5.34, 6.70]` — matches the
already-validated `[5.35, 6.69]` reference. `ChiSquaredQuantile.
MatchesKnownNeesBounds` encodes this check as a regression test.

**`McRunner.EkfNeesConsistencyMatchesPhase1Result`** reproduces Phase 1's
exact NEES consistency test (N=100, 500 steps, dt=10s, ISS orbit, GPS
σ=10m, Q_pos=1m, Q_vel=0.01 m/s) end-to-end through the new threaded
`mc_runner.cpp` path, using `chi_squared_quantile()` for the bounds instead
of hardcoded constants — passing this is strong evidence that both the
threaded run distribution and the general bounds formula are correct, since
it's an independent re-derivation of an already-known-good result through a
different code path.

Verified TSan-clean (`-DENABLE_TSAN=ON`, all `McRunner.*` tests) — expected,
since there's no shared mutable state between threads during the parallel
phase to race on in the first place, but worth confirming rather than
assuming.

8 tests in `test_mc_runner.cpp`: 2 for `chi_squared_quantile` (matches
known bounds; median ≈ dof), output array sizing, determinism given a fixed
seed, `n_runs` not evenly divisible by `k_mc_threads` (e.g. n_runs=3 with 4
threads — some threads get zero runs, must not crash), KF and UKF
`FilterKind` selection produce finite output, and the Phase 1 NEES
reproduction above.

**Benchmark (`bench_monte_carlo.cpp`):** full `run_monte_carlo()` campaign,
N=1000 runs × 1000 steps, EKF, 4 threads: **141.6 ms**, consistent across
repeated runs — 5.7x inside the `< 800 ms` CLAUDE.md §13 target. Cross-
checked against a rough single-thread extrapolation (single EKF
predict+update ≈0.26 μs × 1000 runs × 1000 steps ≈ 260 ms of pure filter
compute alone, before the RK4 truth propagation and GPS sampling each step
add to that) — landing at 142 ms across 4 threads is consistent with real
parallel speedup, not just a fast single-thread baseline, though this is an
estimate rather than a controlled 1-vs-4-thread measurement, since
`k_mc_threads` is a fixed internal constant rather than an exposed
parameter this session.

### Step 3 — WebGL2 renderer (`earth.ts`, `orbit.ts`, `covariance.ts`, `panels.ts`)

Raw WebGL2, no three.js or other 3D library — CLAUDE.md §4/§20 is explicit
that this is the same approach Figma uses for its rendering engine, not a
wrapper library, and the coding convention "all GL state changes in
renderer/ files only" implies direct GL calls are the intended style. Added
a small shared `renderer/gl_utils.ts` (not in CLAUDE.md's literal repo
layout, but `earth.ts`/`orbit.ts`/`covariance.ts` all need the same
shader-compile boilerplate and 4x4 matrix math, and writing that three
times would violate the project's own simplification rule) — `mat4Identity`/
`mat4Multiply`/`mat4Perspective`/`mat4LookAt`, `compileShader`/
`createProgram`, and `SCENE_SCALE` (1 scene unit = 1 Earth radius, so
`earth.ts`'s unit sphere and `orbit.ts`/`covariance.ts`'s ECI-meter
positions all composite in one consistent scene).

**`earth.ts`:** lat/lon UV sphere (48×96 bands), vertex+fragment shaders
with Fresnel-based atmosphere rim glow (`pow(1-dot(N,V), 3)`, CLAUDE.md
§12). Texture loading is async (`Image.onload`/`onerror`) and falls back to
a flat ocean-blue color with the same Fresnel shading on failure, so the
renderer never shows a blank/crashed sphere even if the texture is missing
or fails to load.

**`earth_8k.jpg` texture — sourced after explicit user confirmation, with
two real mistakes caught before it actually worked.** The asset wasn't in
the repo; per the standing instruction not to guess/generate URLs, this
was raised to the user directly rather than picked silently. User chose
"fetch a known NASA Blue Marble URL." Three specific Earth Observatory URLs
recalled from memory (`eoimages.gsfc.nasa.gov/.../world.topo...jpg`,
`.../world.200401...jpg`, etc.) all 404'd on `curl` — confirms the
"don't guess URLs" instinct was right; instead used `WebSearch` +
`WebFetch` against `visibleearth.nasa.gov` to find the actual current
asset URLs, verified each with `curl -I` before downloading.

*Mistake 1:* the first URL found
(`.../BlueMarble_2005_Afr_03_lrg.jpg`) downloaded fine (200, valid JPEG)
but turned out to be a pre-rendered orthographic *photo of a globe*
(Africa-centered, transparent/white background, square 3735×3735) — not
an equirectangular map. Caught by actually opening the downloaded image
before wiring it in: a sphere already photographed as a sphere can't be
UV-mapped onto another sphere without visible distortion. Went back to
NASA's Blue Marble "base map" page specifically and found the real
equirectangular asset: `.../bmng-base/january/world.200401.3x5400x2700.jpg`
(5400×2700, exactly 2:1 — the correct projection for a lat/lon UV sphere).

*Mistake 2:* even with the right asset in `web/public/earth_8k.jpg`,
the renderer kept reporting "failed to load" — i.e. the existing
fallback path silently went down the *wrong fork* convincingly enough to
look like a still-missing file. Root cause this time: a **service worker
left registered in the test browser profile from earlier in this
session**, still serving its `orbitforge-v1` cache's stale recollection
of `/earth_8k.jpg` from before the file existed (a 200-with-`text/html`
SPA-fallback response Vite had served back then) — confirmed via
`navigator.serviceWorker.getRegistrations()` showing an active
registration despite `main.ts`'s `import.meta.env.DEV` guard (added a few
steps earlier in this same phase) correctly preventing *new*
registrations; the guard doesn't retroactively unregister one a prior
test session already installed. `curl` against the dev server directly
returned the correct image the whole time — this confirms the earlier
Step 4 SW-caching gotcha is a recurring category, not a one-off, when
iterating against a long-lived browser profile during manual/automated
browser testing in this repo: **always check
`navigator.serviceWorker.getRegistrations()` and `caches.keys()` first**
when a fix "isn't taking effect," before assuming the fix is wrong.
Unregistering the stale SW and clearing its cache made the texture load
immediately; verified via screenshot — North America with visible snow
cover, oceans, and the Fresnel atmosphere rim all rendering correctly.

**`orbit.ts`:** one `GL_LINE_STRIP` per path (true/KF/EKF/UKF), each a
capped circular history buffer (`MAX_POINTS_PER_PATH = 2048`) uploaded via
`bufferSubData` only when dirty. CLAUDE.md §12 calls for the true path
dashed; rendered solid white instead — true dashing needs a per-vertex
"distance along path" attribute and discard-based stippling in the
fragment shader, real complexity for a cosmetic distinction already covered
by the true path's distinct color against the blue/teal/orange filter paths.

**`covariance.ts` — known, documented gap vs. the literal spec.** CLAUDE.md
§12 says "eigendecompose P[0:3,0:3] → 3 semi-axes → transform unit sphere."
That needs the *full* 3×3 position covariance block including off-diagonal
terms. `StateFrame` (`engine/include/wasm_api.hpp`, decided in Phase 2)
only transmits the covariance **diagonal** (`kf_cov_diag` etc., 6 doubles:
3 position + 3 velocity variances) — a deliberate bandwidth/simplicity
choice made before this renderer existed. A diagonal matrix's eigenvectors
are exactly the coordinate axes and its eigenvalues are the diagonal
entries themselves, so what `covariance.ts` actually renders is an
ECI-axis-aligned wireframe ellipsoid (semi-axes `3·sqrt(variance)` per
axis), not a body-frame-oriented one from a true eigendecomposition. This
is the right implementation *given the data StateFrame carries* — doing the
literal spec would require changing `StateFrame` to carry off-diagonal
terms too, a Phase 2 decision I didn't revisit this session. Documented in
the file's header comment so nobody mistakes the axis-aligned ellipsoid for
a bug later.

**`panels.ts`:** added `chart.js` (^4.4.7) as the first real npm runtime
dependency (previously only `vite`/`typescript` as devDependencies) — `npm
install` came back with 0 vulnerabilities. Four streaming line charts
(position error norm, velocity error norm, covariance trace, NIS), each
fed from a capped 300-point ring buffer, `chart.js animation: false` so
`chart.update()` calls don't fight the app's own 60fps loop. NIS bounds use
the standard `chi2(3, 0.025) = 0.216` / `chi2(3, 0.975) = 9.348` table
values directly — these are for a **single live run** (N=1, measurement
dim=3), not the N-averaged bounds `mc_runner.cpp`'s `chi_squared_quantile()`
computes for a Monte Carlo campaign (CLAUDE.md §8); the two are different
statistics and don't share a formula instance in this codebase yet.

**Dead end — flexbox layout bug found via the smoke test, not by
inspection.** First draft of the `#layout` CSS used `#scene-canvas { flex:
2 1 auto; width: 100%; ... }` and `#panels { flex: 1 1 320px; min-width:
280px; ... }`. Screenshotted via a temporary Playwright-driven smoke test
(see below) and the panels sidebar was rendering 288px wide but positioned
*starting past the right edge of the 1400px viewport* — total layout width
of 1371+288=1659px overflowing the container. Root cause: a flex item's own
`width: 100%` together with `flex-basis: auto` creates a sizing dependency
on the flex container's distributed space that doesn't resolve the way a
non-flex `width:100%` would — a known flexbox gotcha. Fixed by switching to
the standard deterministic sidebar pattern: `flex: 1 1 0%; min-width: 0`
for the canvas (no content-based basis) and `flex: 0 0 320px` (fixed,
non-growing, non-shrinking) for the sidebar. This was caught entirely by
actually rendering it in a browser — `tsc --noEmit` and `vite build` both
stayed green throughout, since this is a pure CSS/runtime layout issue, not
a type error.

**Verification approach:** no WASM build locally (same constraint as
Phase 2 — no Emscripten toolchain), so the production `main.ts` →
worker → WASM → ring buffer pipeline could not be exercised end-to-end.
What *could* be verified: the renderer modules themselves are
framework/WASM-agnostic exports, so a temporary harness
(`web/test-renderer.html` + `.ts`, deleted after use, not committed) fed
synthetic `StateFrame`-shaped data directly into `EarthRenderer`/
`OrbitPathRenderer`/`CovarianceEllipsoidRenderer`/`PanelManager` and was
loaded in a real browser via Playwright. Confirmed: Earth sphere renders
with fallback color and visible Fresnel rim glow; orbit path renders as a
visible line strip; covariance ellipsoids render as small markers along
the path (correctly tiny — 3σ at a ~20km position std relative to a
6378km Earth radius is genuinely a small fraction of the scene, which is
accurate, not a bug); all four Chart.js panels render live, updating data
including the NIS dashed bound lines. Console showed only the expected
texture-404 fallback warning and an unrelated favicon 404. This verifies
the renderer code is runtime-correct in a real browser; it does not verify
the WASM-driven production data path, which remains untested locally and
deferred to CI/manual testing once a WASM build is available.

### Step 4 — `ui/scenario_editor.ts` + `ui/fault_panel.ts`

`ScenarioEditor`: satellite `<select>` populated from `tle_feed.ts`'s
`PRESETS`, a paste-TLE fallback textarea, GPS σ / sim-speed range sliders,
J2/Drag/SRP checkboxes, and Run/Pause/Reset buttons — constructs a
`ScenarioConfig` and calls the caller-supplied `postToWorker` callback,
never `ccall` directly (CLAUDE.md §20). `FaultPanel`: one button per
`FaultType`, default magnitudes from CLAUDE.md §9's table (GPS spike 500m,
maneuver 5 m/s, drag error +50%, etc.). Every fault button uses
`onsetT: 0` — `Simulation::step()` applies a fault once
`t_now >= onset_t`, and since `sim_time` only increases from 0, `onset_t=0`
always means "apply on the next tick" regardless of how long the
simulation has already run, so there's no need for the UI to track or read
back the current sim time at all.

**Found and fixed a real, currently-broken bug in the pre-existing
`tle_feed.ts` scaffold: the CelesTrak endpoint it called
(`/satcat/tle.php?CATNR=`) was deprecated in 2020 and removed in 2022.**
Confirmed live via `curl` — it now returns an HTML deprecation notice, not
a TLE, which is exactly what made `parseTle()` throw "Invalid TLE line
designators" (the HTML's first line obviously doesn't start with `1 `).
This wasn't something a passing typecheck could have caught — `tle_feed.ts`
typechecked fine throughout, the bug only showed up by actually loading the
page and watching the satellite picker fail. Fixed by switching to
CelesTrak's current GP-data API:
`https://celestrak.org/NORAD/elements/gp.php?CATNR={id}&FORMAT=TLE`
(verified via `curl` to return a clean 3-line name+TLE response, 70-char
lines).

**Dead end — the fix above didn't visibly work on the first reload, and
the cause was the project's own service worker.** After editing
`tle_feed.ts`, reloading still showed the same stale error — even after a
full dev-server restart. Tracked it down by inspecting
`navigator.serviceWorker.getRegistrations()` and `caches.keys()` in the
running page: `sw.ts`'s cache-first strategy (`orbitforge-v1`) had already
cached the old, broken module response and kept serving it regardless of
server-side changes. Unregistering the SW and clearing its cache made the
fix appear immediately. Fixed at the source rather than just clearing the
cache once: `registerServiceWorker()` in `main.ts` now skips registration
entirely when `import.meta.env.DEV` is true, so this can't recur during
local development (added `"types": ["vite/client"]` to `tsconfig.json` for
`import.meta.env`'s type). The SW still registers normally in production
builds, which is the only place its offline-caching behavior is wanted.

**Verification:** same Playwright-driven approach as Step 3, but this time
against the real `main.ts` (not a synthetic harness) — `npm run dev`,
navigate, read `.status-line` text via `page.evaluate`, confirmed it
progresses from "Fetching TLE from CelesTrak..." to "Loaded ISS (ZARYA)"
against the live CelesTrak API (real network call, not mocked). Console
showed only the expected texture-fallback and manifest-icon warnings, no
errors, after the SW/cache state was cleared once. The full Run → worker →
WASM path remains unverified (no Emscripten locally, same as every prior
WASM-touching step this phase) — clicking Run posts `init`+`start` to the
worker, which will attempt to load the non-existent `/orbitforge.js` and
fail; this is expected and matches the documented WASM-deferred-to-CI
posture, not a new gap introduced here.

### Step 4b — UI polish pass + camera drag fix (user feedback)

User feedback after seeing the running app: UI "could be so much more
responsive, cleaner, pop more," and dragging the Earth "can feel normal."
Two separate problems, addressed separately.

**Camera drag — root-caused, not just tweaked.** The arcball camera
(`main.ts`'s `OrbitCamera`) used `mousedown`/`mousemove`/`mouseup` with no
`preventDefault()`. That meant every drag was racing against the browser's
own native text-selection/drag-image gesture — the actual cause of the
"doesn't feel normal" complaint, not the rotation math itself. Also: the
`mousemove` listener was attached to the canvas, not `window`, so dragging
past the canvas edge silently stopped tracking; and there was no cursor
feedback or release inertia, both of which read as "stiff" even when the
rotation itself was working.

Rewrote using the Pointer Events API with `setPointerCapture` (keeps
receiving move events even if the pointer leaves the canvas mid-drag —
the old approach's edge-tracking bug), `e.preventDefault()` on
`pointerdown`/`pointermove` (stops the native drag/selection gesture from
ever engaging), `grab`/`grabbing` cursor feedback, and a small inertia
system: the last per-event angular delta becomes a velocity that decays
by `0.92`/frame via a new `camera.update()` call added to the render loop,
so releasing mid-drag continues the rotation briefly instead of stopping
dead — verified via Playwright (`mouse.down` → `mouse.move` → `mouse.up`,
screenshot immediately after release vs. ~600ms later) showing the globe
visibly continuing to rotate after release and decaying smoothly. Also
verified zero `window.getSelection()` text after a drag, confirming the
old selection-fighting bug is actually gone, not just less visible.

**Visual polish.** Invoked the `ui-ux-pro-max` skill for a dark
technical-dashboard design system (`Inter` + `Fira Code` pairing — sans
for UI text, mono for data/numbers, matching the "dashboard, data,
analytics, precise" mood) rather than guessing at colors/spacing myself.
Applied as CSS custom properties (semantic tokens: `--bg`, `--surface`,
`--border`, `--text-muted`, `--accent-blue/teal/orange`, `--radius`,
`--transition`) instead of the scattered hardcoded hex values from the
first pass. Concrete changes: a real topbar with a brand mark (small glow
dot) instead of a bare status line; each chart wrapped in a `.panel` card
with its own `<h4>` title (`POSITION ERROR [m]`, etc.) so Chart.js legends
could drop the repeated title-per-series text and just say `KF`/`EKF`/
`UKF` — three chart panels' worth of redundant text removed, which alone
accounts for a lot of the "pop more"/cleaner feedback; styled range-input
sliders (custom track/thumb, since the unstyled native slider was one of
the flatter-looking elements); button hover/active/focus-visible states
with 150-200ms transitions (`prefers-reduced-motion` respected — all
transitions disabled under that media query); a responsive breakpoint at
900px that stacks the layout vertically instead of a fixed flex split that
didn't adapt.

**Color consistency fix.** The KF/EKF/UKF colors were defined three
separate times with slightly different values: `orbit.ts`'s
`PATH_COLORS`, `main.ts`'s inline covariance-ellipsoid color arguments,
and `panels.ts`'s `FILTER_COLORS` for Chart.js — close but not identical
RGB triples in each place, so the "same" filter could read as a subtly
different shade in the 3D scene vs. the charts. Consolidated into one
`FILTER_COLOR_RGB` constant in `gl_utils.ts`, imported by both `orbit.ts`
and `main.ts`; `panels.ts` keeps its own copy in CSS-string form (Chart.js
wants `'rgb(r,g,b)'` strings, not normalized floats) but the numbers now
match the same `--accent-*` CSS tokens by construction, not by
coincidence.

**Dead end — the first version of the 900px breakpoint had a real overlap
bug, caught only by measuring `getBoundingClientRect()`, not by looking at
a screenshot.** First draft: `#layout{flex-direction:column}`,
`#panels{flex:0 0 360px; ...2x2 grid...}`. A screenshot at 700px width
looked like the Scenario card was overlapping the bottom two chart panels.
Measuring the actual boxes confirmed it: `#panels` (360px, non-shrinking)
plus `#scene-canvas` (which reused its row-layout `flex: 1 1 0%` rule
unchanged) inside `#layout`'s 455px box. The default `min-height: auto` on
a flex item can lock a `<canvas>` to its last-rendered intrinsic height
when flex-direction flips from row to column — `min-width: 0` (already
present, needed for the row case) doesn't cover the column case, so
`scene-canvas` refused to shrink and both children overflowed `#layout`'s
box, with `#panels` visually landing on top of `#controls` underneath.
Fixed by giving `#scene-canvas` a fixed `280px` height at this breakpoint
instead of fighting for flex space, and switching the whole mobile layout
from "fit everything into one 100vh viewport" to a normal scrolling
document (`html, body, #app { height: auto; min-height: 100% }`) — a more
robust pattern for a data-dense dashboard on a small screen than cramming
four chart panels and two control cards into whatever's left after a 3D
viewport. Re-verified with the same `getBoundingClientRect()` check (no
overlaps) plus a full-page screenshot at 700×900 showing all 4 panels in
a legible 2×2 grid, the globe at a reasonable size, and both control cards
stacked below with normal page scroll. Re-checked 1440×900 afterward to
confirm the media-query-only change didn't touch desktop.

Typecheck and `vite build` both clean throughout.

### Step 4c — starfield backdrop (user feedback)

User: make the scene background "look starry like the solar system than
just black... reflective of our realistic solar system." Read as wanting
a believable deep-space backdrop, not a literal multi-planet solar-system
renderer — OrbitForge stays an Earth-orbit estimation tool per CLAUDE.md
§18.

Added `renderer/starfield.ts`: a procedural starfield, not a real star
catalog (no RA/Dec data) — ~3500 dim background stars from uniform-sphere
point picking (not uniform lat/lon, which visibly clusters at the poles),
a denser/brighter band along a tilted great circle standing in for the
Milky Way, and 40 brighter foreground stars, all from a fixed seed
(mulberry32 PRNG) so the sky is stable across reloads rather than
reshuffling on every refresh. Rendered as `GL_POINTS` with a
circular-falloff fragment shader (square points read as "pixelated," not
"stars"). Added `mat4StripTranslation()` to `gl_utils.ts` to zero a view
matrix's translation column, rendering the star sphere as though at
infinite distance — without it, zooming toward Earth would make the
stars visibly drift/approach, which reads as wrong (real stars don't get
closer when you zoom in on a nearby planet). Rendered first each frame,
before Earth, so the depth test naturally occludes stars behind the
globe — confirmed via screenshot that none show through the planet.

**Milky Way band — tuned twice, accepted as subtle rather than chased
further.** The band is rejection-sampled (keep points within a threshold
angular distance of a tilted plane through the origin) with
brightness/size increasing toward the plane. Verified the geometry is
sound, including aiming the camera at an angle computed to be
perpendicular to the band's normal vector (so the view looks straight
down the band, the angle that should make it span the widest swath of
sky) and screenshotting from exactly that angle. Even after boosting
density (2200→6000 points) and brightness (0.08-0.43→0.35-0.80, scaled by
falloff) twice, the band reads as texture rather than an obviously
distinct bright streak. Decided not to keep iterating on this alone: the
resulting sky — dense, varied-brightness, varied-size scattered stars —
already reads as a convincing, realistic deep-space backdrop, which was
the actual ask. A sharper band is a nice-to-have, not a gap, and "does
this look good enough" is a visual call better left to the user's own
eyes than to further solo iteration.

Typecheck clean after both tuning passes.

### Step 5 — Monte Carlo wired into the WASM API + UI

Previously deferred because CLAUDE.md §21's `run_monte_carlo(n_runs, seed)`
signature carries no `n_steps`/`dt`/filter-kind, and `ScenarioCfg` has no
`duration` field — there was no way to know the real result shape without
deciding those gaps first. Decided them now, each documented at the
decision site rather than left implicit:

- **Always EKF.** KF is the intentionally-divergent demo filter (CLAUDE.md
  §6) — running a consistency campaign against a filter that's *supposed*
  to diverge defeats the point. UKF is ~2.5x the per-step cost
  (`docs/benchmarks.md`) for the same consistency question EKF already
  answers. CLAUDE.md §21 gives `run_monte_carlo` no filter-selection
  argument, so EKF is the one defensible default, not an arbitrary one.
  Documented on `Simulation::run_monte_carlo`'s declaration in
  `wasm_api.hpp`.
- **`n_steps=500, dt=10s` fixed, not configurable.** Matches the
  Phase-1-validated setup (`test_filter_consistency.cpp`,
  `McRunner.EkfNeesConsistencyMatchesPhase1Result`) — one
  ISS-orbit-scale campaign (~83 min sim time) per run. Not configurable
  because nothing in the §21 API or `ScenarioCfg` carries a duration to
  configure it with; inventing a new parameter not in the spec felt like
  the wrong kind of gap to fill silently.
- **MC uses the scenario's *initial* true state, not the live one.**
  Added `x_true_initial_` to `Simulation`, snapshotted once in
  `init_scenario()` alongside the mutating `x_true_`. Running MC after the
  live sim has been stepping for a while shouldn't make the campaign start
  from wherever the live trajectory happens to be — a Monte Carlo
  consistency check is about the filter's response to a *given* initial
  condition, not a moving target.
- **`run_monte_carlo()` pauses any running live simulation first.** Real
  concurrency hazard, not a hypothetical one: the engine's MC runner
  spawns its own `monte_carlo::k_mc_threads` (4) worker threads
  (`mc_runner.cpp`), and the WASM build's `PTHREAD_POOL_SIZE` is sized to
  match exactly that. If the live sim's own background thread
  (`Simulation::run_loop`) were also running, that's 5 threads needed
  against a 4-slot pool — a real deadlock risk under Emscripten pthreads,
  not just inefficiency. `pause()` first removes the 5th contender instead
  of trying to grow the pool to cover a case that's avoidable.

**Engine-side data shape — added a real histogram, not a synthetic one.**
CLAUDE.md §12's MC panel mockup lists `[Histogram] [RMS table]
[NEES/NIS consistency]` as three separate widgets, but `MCStats` (as it
existed) only carried per-step *aggregates* across runs — nothing to
histogram per-run. Rather than fake one, extended `MCStats` with
`final_pos_err` (size `n_runs`, the final-step `|r_true - r_hat|` per run)
and threaded it through `PartialSums`/`run_one`/`run_slice`/
`run_monte_carlo()` in `mc_runner.cpp`, ordered by run index (not thread
completion order — verified by
`McRunner.FinalPosErrIndexedByRunOrderNotCompletionOrder`, which
reproduces one run in isolation and checks it lands at the same index in
a 9-run campaign with uneven thread slices). Also added
`nees_bounds(n_runs)`/`nis_bounds(n_runs)` (CLAUDE.md §8's
`chi2(6N,·)/N` and `chi2(3N,·)/N` formulas) so the UI doesn't need to
reimplement the bounds math — verified against the same `[5.35, 6.69]`
reference `ChiSquaredQuantile.MatchesKnownNeesBounds` already used.

**WASM bindings** (`wasm_api.cpp`): `run_monte_carlo(int n_runs, int seed)`
matches CLAUDE.md §21 exactly. `get_mc_results()` doesn't exist as a
single binding — ccall can't return a struct, so it's split into the same
pointer+count pattern the ring buffer already uses:
`get_mc_n_steps`/`get_mc_n_runs` (counts) plus
`get_mc_rms_pos_ptr`/`get_mc_rms_vel_ptr`/`get_mc_nees_ptr`/`get_mc_nis_ptr`/
`get_mc_final_pos_err_ptr` (raw `uintptr_t` into the `MCStats` stored on
the global `Simulation`, valid until the next `run_monte_carlo()` call)
plus `get_mc_nees_lower/upper`/`get_mc_nis_lower/upper` (scalars). New
native tests: `Simulation.RunMonteCarloProducesCorrectlySizedFiniteResults`,
`RunMonteCarloIsDeterministicGivenSameSeed`,
`RunMonteCarloPausesAnyRunningLiveSimulation`, plus the free-function-API
equivalent — 84/84 native tests passing (was 81).

**Web side**: `worker.ts` gained a `run_monte_carlo` request type — calls
`ccall('run_monte_carlo', ...)` (blocks the *worker* thread for the
campaign's duration, not the main UI thread, exactly the architecture
CLAUDE.md §4 separates threads for), then copies the result arrays out of
`module.HEAPF64` via the pointers immediately, before any other ccall can
reallocate them, and posts a plain-data `mc_results` message back.
`ui/mc_results.ts` is a `<details>`-based collapsed-by-default panel
(native disclosure widget — keyboard-accessible, no extra JS needed)
with a Runs slider (100-5000), a bar-chart histogram of
`finalPosErrPerRun`, an actual `<table>` for RMS at 4 time fractions (a
literal table, not another line chart, since the mockup calls out
"table" specifically), and two bounded line charts for NEES/NIS using
*this campaign's* `neesLower/Upper`/`nisLower/Upper` — distinct from
`panels.ts`'s fixed single-run `chi2(3)` bounds, since MC bounds
genuinely depend on `n_runs`. Verified visually via Playwright with
injected synthetic `MCStats` (no local WASM build to drive it for real):
expand/collapse works, histogram/table/both charts render with sane
values, responsive 2-column layout at the 900px breakpoint.

Typecheck and `vite build` both clean.

### Step 6 — All 5 satellite presets verified end-to-end

The TLE-fetch/parse/UI portion of this is real-network-testable without a
WASM build (only the downstream physics-engine path is blocked on the
missing Emscripten toolchain), so it was actually run rather than left as
an open item.

**Found and fixed a real bug: STARLINK-1007 (NORAD 44713) has deorbited.**
`curl`'ing CelesTrak's GP-data API for all 5 presets' NORAD IDs (the same
endpoint `tle_feed.ts` calls, fixed in Step 4) returned valid TLEs for 4 of
5 — `CATNR=44713` returned `No GP data found`, CelesTrak's response for an
object no longer tracked. Confirmed via CelesTrak's `GROUP=starlink` GP
listing that STARLINK-1007's batch-mate STARLINK-1008 (NORAD 44714) is
still active with a closely matching orbit (53.15° inclination, 15.50
rev/day mean motion — consistent with the original "550 km, 53°" preset
description). Swapped the preset in `tle_feed.ts` to NORAD 44714 with the
same "550 km, 53°, high A/m, drag dominates" description, which still
applies.

**Verification approach:** ran the actual `parseTle()`/`fetchTleByNorad()`
logic (not a reimplementation) against live CelesTrak responses for all 5
corrected NORAD IDs via Node 24's native TypeScript stripping
(`node -e "import('./tle_parser.ts')..."`), confirming sane orbital
elements for each (ISS 51.6°, Starlink-1008 53.15°, GPS BIIR-2 55.1°/12hr
mean motion, GOES-16 0.34° near-equatorial GEO, debris 74.2° high
inclination). Then drove the real `ScenarioEditor` UI in a browser via
Playwright — selected each of the 5 dropdown options in turn and confirmed
the status line reaches `Loaded <name>` for all five (initial test run was
confounded by page navigations aborting in-flight fetches mid-test,
producing false "still fetching" reads; re-ran cleanly against a single
stable page load with the test polling the status line to completion
rather than reading it once after a fixed delay).

This closes the "all 5 satellite presets working end-to-end" item to the
extent verifiable without Emscripten: TLE fetch, parse, and UI wiring are
confirmed correct against live data for all 5. Clicking "Run" still
requires the WASM module (`/orbitforge.js`), which doesn't exist locally —
that remains deferred to CI, unchanged from every previous WASM-touching
step this phase.

Typecheck and `vite build` both clean.

### Step 7 — Phase 4 triage: deploy.yml, math.md, benchmarks.md, README, architecture.md, PWA icons

Before editing anything, read all five Phase 4 checklist items against
what actually exists, to avoid redoing work that was already done in
earlier phases:

- `deploy.yml` already matches CLAUDE.md §15 exactly (checkout, Emscripten
  SDK cache+install, `build_wasm.sh`, Vite build, `cloudflare/pages-action`).
  Nothing to change in the workflow file itself — what's missing is
  account-side: the `CLOUDFLARE_API_TOKEN`/`CLOUDFLARE_ACCOUNT_ID` repo
  secrets and an actual Cloudflare Pages project named `orbitforge`, both
  of which only the repo owner can create.
- `docs/math.md` is already comprehensive (EOM, RK4/RK45 with full
  Dormand-Prince tableau, two-body + J2 Jacobians with all 9 entries,
  SR-UKF sigma points/Cholesky update, GPS/IMU/magnetometer models, NEES
  MC methodology). Cross-checked every `math.md §...` citation appearing
  in engine source comments (21 total, across `eom.cpp`, `ekf.cpp`,
  `ukf.cpp`, `magnetometer.cpp`, etc.) against the file — every citation
  points at a section that actually exists and matches. No edits needed.
- `docs/benchmarks.md` is already complete for what's measurable locally:
  full Phase 1 native numbers (RK4 0.149µs, EKF 0.247µs, UKF 1.292µs, 3
  filters/tick 1.655µs, MC 1000×1000 141.6ms, ring buffer 5.74×10⁸/sec,
  SoA +484.9% over AoS), with the WASM-overhead row honestly marked
  "not measured — no local Emscripten toolchain" rather than faked. No
  edits needed.
- `README.md` was genuinely stale: Phase 3 checkbox still unchecked
  despite Phase 3 being fully done (Steps 1–6 above), and an "Architecture"
  section linking to `docs/architecture.md`, which didn't exist. Fixed
  both, added a real "Web (TypeScript + WASM)" build section (previously
  the README only documented the Phase 1 native C++ build), and added a
  real screenshot of the running app.
- `docs/architecture.md` didn't exist. Created it as a public-facing
  companion to CLAUDE.md's architecture sections (thread model, TLE→filter
  data flow, ring buffer, pool allocator, SoA Monte Carlo, fault
  injection) — written from and cross-checked against the actual
  `wasm_api.hpp`/`ring_buffer.hpp` source, not copied from CLAUDE.md's
  plan, since the plan and the as-built code aren't guaranteed to agree
  in every detail (CLAUDE.md itself is gitignored and never shipped, so a
  public architecture doc needs to stand on its own anyway).
- `web/public/manifest.json` references `icon-192.png`/`icon-512.png`,
  neither of which existed — confirmed via a Playwright console check in
  an earlier session ("Error while trying to use the following icon from
  the Manifest... Download error or resource isn't a valid image") and
  reconfirmed missing via `ls`. No image library is available locally
  (no Pillow, no ImageMagick/`convert`/`rsvg-convert` — only `sips`, which
  converts but doesn't generate from nothing). Wrote a small
  `scripts/gen_icons.py` that hand-encodes a PNG (manual IHDR/IDAT/IEND
  chunks via stdlib `zlib`/`struct`, no dependencies) drawing a simple
  planet+orbit+satellite glyph in the manifest's own background/theme
  color (`#0a0a0f`) plus the renderer's orbit-path accent colors.
  Generated both sizes, verified with `sips -g pixelWidth -g pixelHeight`
  and `file` that both are valid 192×192/512×512 RGBA PNGs, then
  reconfirmed via a fresh Playwright page load that the manifest-icon
  console warning is gone.

**Screenshot verification:** started the real `vite dev` server (not a
synthetic test harness) and loaded the actual production `index.html` —
confirmed via console messages that a fresh navigation has zero
errors/warnings (the manifest-icon warning seen in scrollback was from
page loads before the icon files existed; an unrelated `MCResultsPanel
appendChild null` error in scrollback was leftover from a prior session's
stale HMR-reloaded module, not from a clean load — reproduced a clean
load immediately after to confirm it doesn't recur). Captured
`docs/screenshots/main-view.png` from this clean load: Earth renderer,
scenario editor, fault injection panel, and the four chart panels are all
real, current UI — not mocked. Charts are empty because no WASM module is
present locally to drive the simulation loop, which is the same
constraint noted for every WASM-touching step since Phase 2.

### Next

Outreach (posting to r/spacex, r/aerospace, AIAA listservs, Spaceshot
Rocketry) is the one remaining Phase 4 checklist item, and it's a manual,
external action for the project owner — not something to automate.
Otherwise Phase 4 is done to the extent verifiable without a real
Emscripten/Cloudflare account setup. A demo GIF would need a live WASM
build actually driving the sim loop to be worth recording; that's the
next thing worth doing once Emscripten is available (CI, or installed
locally).
