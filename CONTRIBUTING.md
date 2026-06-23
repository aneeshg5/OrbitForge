# Contributing to OrbitForge

OrbitForge is a learning tool first. Found a place where it doesn't match what you'd expect from the theory? Open an issue, even if you're not sure it's a bug. Have an idea, even half-formed? Open a discussion. No contribution is too small.

## Ways to contribute

- **Bug:** use the bug report template.
- **Idea:** use the feature request template, you don't need to build it yourself to suggest it.
- **Not sure yet?** [Discussions](https://github.com/aneeshg5/OrbitForge/discussions) is lower-friction than an issue.
- **PR:** small fixes (typos, clear bugs, docs) can just be a PR. Anything that changes behavior or touches the filter math, open an issue first so we're aligned before you do the work.

## Building and testing

Full setup is in the [README](README.md#getting-started). Short version:

```bash
# C++ engine: build + run all 112 tests
cmake -B build -DCMAKE_BUILD_TYPE=Debug engine/
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure

# Web frontend: typecheck
cd web && npm install && npx tsc --noEmit
```

Engine PRs should keep tests passing under the sanitizer builds CI runs (ASan/UBSan, TSan). Web PRs should keep `tsc --noEmit` clean.

## A few conventions

No separate style guide, match the surrounding code. Two things worth knowing up front: C++ is strictly C++17 (Emscripten constraint), and comments are intentionally sparse, reserved for genuinely non-obvious things like a `docs/math.md` citation or memory-ordering rationale, not narration of what the code does. Please don't add that style back in a PR.

Want a starting point instead of a from-scratch idea? The engine has a few known simplifications worth picking up: NRLMSISE-00 vs. the current 7-band atmosphere model, J2-only gravity (no J3-J6), no active attitude control (reaction wheels/magnetorquers), no multi-satellite scenarios. These are real gaps, not busywork.
