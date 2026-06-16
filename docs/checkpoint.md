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

## What's next — Phase 2

Per `CLAUDE.md` §17: `memory/ring_buffer.hpp` (lock-free SPSC,
64-byte-aligned, TSan-clean), `memory/pool_alloc.hpp` (fixed-block pool,
64-byte alignment), `wasm_api.cpp` (Emscripten bindings), then the
`scripts/build_wasm.sh` pthread+SIMD128 build and `web/src/worker.ts`
100 Hz driver loop. The ring buffer and pool allocator tests from
CLAUDE.md §14 (`test_ring_buffer.cpp`, `test_pool_alloc.cpp`) belong here,
not in Phase 1 — they don't exist yet because their headers don't exist
yet.
