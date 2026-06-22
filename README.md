# OrbitForge

Real satellite orbits, real Kalman filters, running entirely in your browser.

[orbitforge.dev](https://orbitforge.dev)

![CI](https://github.com/aneeshg5/OrbitForge/actions/workflows/ci.yml/badge.svg)

![OrbitForge main view](docs/screenshots/main-view.png)

## What this is

Paste in a TLE for any tracked object in orbit, or pick one from the live CelesTrak feed, and watch three different filters estimate its position and attitude from noisy sensor data, in real time, with nothing to install and no backend server. The orbital mechanics, the filters, and the sensor models are all C++ compiled to WebAssembly. The browser only handles rendering.

I built this because I wanted to actually watch filter divergence happen instead of just reading about it in a textbook. Orekit will run an EKF against a real orbit, but it's a Java library, not something you hand to someone and say "click this." nyx does orbital propagation in WASM but has no state estimation at all, just propagation. orbidet and poliastro are Python notebooks, correct but static, and not something you'd show a non-technical person. None of the existing tools let you inject a fault mid-run and watch three filters respond to it differently.

## The actual engineering problem

The interesting part of this project isn't the orbital mechanics, it's getting a 100Hz physics simulation, three Kalman filters, and a Monte Carlo engine to run inside a browser tab without freezing the UI thread.

The simulation runs on its own Web Worker, ticking at a fixed 100Hz independent of how fast the browser can actually render. Every tick it writes a snapshot of the current state, true trajectory, all three filter estimates, covariances, NIS, into a fixed-size ring buffer backed by a SharedArrayBuffer. The render loop reads out of that buffer at 60fps. Producer and consumer are different threads and never block each other: the ring buffer uses atomic head and tail pointers, padded onto separate cache lines so the two threads aren't invalidating each other's cache line on every push and pop.

Monte Carlo campaigns are the other place this mattered. Running 5000 realizations of a filter sequentially in JavaScript would be unusably slow. The campaign instead gets split across a 4-thread pool inside WASM, real OS threads via Emscripten pthreads, so a 5000-run, 500-step campaign (2.5 million filter updates) finishes in under two seconds. That call blocks the worker thread for its entire duration, so I added a separate atomic counter that the main thread polls directly off the shared heap, letting the UI show live progress without waiting on a response message from the worker.

## The filters

Three filters run side by side against the same noisy measurements:

- **KF**: a linear Kalman filter, position and velocity only, linearized around a fixed reference orbit. It's deliberately the worst filter here. Real orbits curve, the KF's linear model can't keep up, and watching it diverge over time is the entire point of including it: it's the control group.
- **EKF**: extended Kalman filter, 12-state (attitude error, angular velocity, position, velocity), using a multiplicative error-state formulation for attitude so the filter never has to linearize through a four-parameter quaternion's unit-norm constraint.
- **UKF**: the same 12-state model as the EKF, but propagated through unscented sigma points instead of an analytic Jacobian, in square-root form so the covariance can't drift non-positive-definite from floating point error.

KF never got a 6DOF upgrade. That was a deliberate decision, not a missing feature. Giving it an attitude estimate would mean building a second, separately bad linearization just so all three panels look symmetric, and it would muddy the actual story being told (is it diverging because of orbital or attitude linearization?). It stays exactly as naive as it always was.

## What's modeled

True trajectory propagation uses RK4 with J2 oblateness, atmospheric drag, and solar radiation pressure, each individually toggleable. Attitude follows torque-free rigid body rotation under Euler's equation. Sensors are GPS (position, ECI to ECEF), a gyroscope, and a magnetometer using an IGRF dipole approximation, each with configurable noise. None of this is meant to replace GMAT or a real flight dynamics suite. Gravity is J2 only, not full spherical harmonics, and the atmosphere model is a 7-band exponential fit, not NRLMSISE-00. Every Jacobian used by the EKF and UKF is derived by hand in docs/math.md, partly so I could check my own work before trusting the filter output.

## Fault injection

You can inject a GPS spike, a GPS dropout, an unmodeled maneuver, a drag coefficient error, or a persistent GPS bias at a chosen point in simulated time, and watch the three filters respond differently. The fault only changes what the filter measures, never the true trajectory, which is what actually makes the comparison meaningful. You're looking at three different estimates of the same ground truth, not three different simulations.

## Monte Carlo

Beyond a single run, you can launch a full consistency campaign: choose the filter, the number of realizations, how many steps each one runs and at what timestep, the process noise, and whether the random seed is fixed or different every time. Results come back as a final position error histogram, an RMS error table, and NEES and NIS consistency charts against their theoretical chi-squared bounds, which is the actual question Monte Carlo answers: is the filter's reported uncertainty trustworthy on average, not just in one lucky or unlucky run.

## Running it

Engine tests, native, no Emscripten required:

```
cmake -B build -DCMAKE_BUILD_TYPE=Debug engine/
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```

Web app:

```
cd web
npm install
npm run dev
```

The dev server expects the WASM build (`orbitforge.wasm`, `orbitforge.js`) already sitting in `web/public/`. That's built separately with `scripts/build_wasm.sh` via Emscripten. See `docs/architecture.md` for the full toolchain and the COOP/COEP headers a SharedArrayBuffer needs to even exist in the browser.

## Status

112 engine tests passing, clean under AddressSanitizer, UndefinedBehaviorSanitizer, and ThreadSanitizer. 8 end to end browser tests covering the run, pause, reset, and Monte Carlo lifecycle. CI builds and tests the engine and the WASM bundle on every push. Deployment to orbitforge.dev is wired up in CI but not live yet, it needs a Cloudflare account on my end.

Further reading: `docs/architecture.md` for the system design, `docs/math.md` for every Jacobian derivation, `docs/benchmarks.md` for measured numbers against target.
