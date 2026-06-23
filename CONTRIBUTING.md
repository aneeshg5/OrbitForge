# Contributing to OrbitForge

OrbitForge is a learning tool first. If you're studying orbital mechanics, Kalman filtering, or GNC, and you find a place where the engine's behavior doesn't match what you'd expect from the theory, that's exactly the kind of thing worth opening an issue over, even if you're not sure yet whether it's a bug or you're missing something. Same goes for "this would help me understand X better" feature ideas.

No contribution is too small. Typos in `docs/math.md`, a confusing UI label, a missing test case, all welcome.

## Ways to contribute

- **Bug report:** something doesn't match the physics/filter theory, crashes, or behaves inconsistently. Use the bug report issue template.
- **Feature idea:** a new sensor model, fault type, filter, perturbation, or visualization. Use the feature request template, you don't need to build it yourself to suggest it.
- **Questions / discussion:** not sure if something's a bug, or just want to talk through an idea before writing it up? [Discussions](https://github.com/aneeshg5/OrbitForge/discussions) is the right place, lower friction than an issue.
- **Pull requests:** for small fixes (typos, clear bugs, small docs improvements), just open a PR. For anything that changes behavior, adds a new model, or touches the filter math, open an issue or discussion first so we're aligned on the approach before you put in the work.

## Ideas if you want a starting point

These are known, intentional simplifications, real gaps where a contribution would genuinely improve the tool, not busywork:

- Atmosphere model is a 7-band exponential approximation, not NRLMSISE-00
- Gravity perturbation is J2 only, not full J2-J6 spherical harmonics
- Solar ephemeris is an analytical approximation, not JPL DE440
- No reaction wheels, magnetorquers, or active attitude control, the rigid body is torque-free
- No multi-satellite scenarios (relative navigation, formation flying)
- The magnetometer Jacobian doesn't differentiate the IGRF field with respect to position (documented simplification in `docs/math.md`)

If you pick one of these up, it's worth opening an issue first so we can talk through scope.

## Building and testing

See the [README](README.md#getting-started) for full setup. The short version:

```bash
# C++ engine: build + run all 112 tests
cmake -B build -DCMAKE_BUILD_TYPE=Debug engine/
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure

# Web frontend: typecheck
cd web && npm install && npx tsc --noEmit

# Web frontend: e2e tests (needs a built WASM bundle, see README step 2)
npm run test:e2e
```

A PR touching `engine/` should keep all native tests passing, including under the sanitizer builds CI runs (ASan/UBSan and ThreadSanitizer separately). A PR touching `web/` should keep `tsc --noEmit` clean and not break the Playwright e2e suite.

## Code conventions

There's no separate style document, the existing code is the style guide, look at a neighboring file before adding a new one. A few things worth knowing going in:

- C++ is strictly C++17, no C++20 features (Emscripten toolchain constraint)
- Eigen matrices in `engine/` are always fixed-size (`Eigen::Matrix<double, 6, 6>`, never `MatrixXd`), the simulation hot path doesn't heap-allocate
- Comments are intentionally sparse throughout the codebase, the convention here is no narration of *what* the code does (the code itself should make that clear), reserved only for genuinely non-obvious things: a `docs/math.md` section citation, memory-ordering rationale on the lock-free ring buffer, or a similar real invariant. Please don't reintroduce broad explanatory comments in a PR.
- TypeScript: no `any`, explicit return types on exported functions, `camelCase`/`PascalCase` per the existing convention

## Reporting something else

Security concern that shouldn't be a public issue? Open a private security advisory from the repo's Security tab, or reach out directly.
