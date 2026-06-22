// TypeScript mirrors of the C++ structs in wasm_api.cpp.
// Keep in sync with engine/src/wasm_api.cpp.

export interface ScenarioConfig {
  tleLine1: string;        // 69-char TLE line 1
  tleLine2: string;        // 69-char TLE line 2
  gpsSigma: number;        // position noise std dev (m)
  imuSigma: number;        // acceleration noise std dev (m/s²)
  enableJ2: boolean;
  enableDrag: boolean;
  enableSrp: boolean;
  dragCoeff: number;       // C_D, default 2.2
  areaToMass: number;      // m²/kg
  qPos: number;            // process noise, position component
  qVel: number;            // process noise, velocity component
  simSpeed: number;        // real-time multiplier
  seed: number;            // RNG seed, -1 = random
  // --- Phase 5: 6DOF (CLAUDE.md §6, §21) ---
  inertiaX: number;        // principal-axis inertia tensor, kg·m²
  inertiaY: number;
  inertiaZ: number;
  gyroSigma: number;       // rad/s, gyro measurement noise
  magSigma: number;        // nT, magnetometer measurement noise
  qAtt: number;            // process noise, attitude error (delta_theta) block
  qOmega: number;          // process noise, angular velocity block
  initOmegaX: number;      // initial true angular velocity, body-frame rad/s
  initOmegaY: number;
  initOmegaZ: number;
}

// Quaternion coefficients in Eigen::Quaterniond::coeffs() order (x,y,z,w) —
// NOT (w,x,y,z). Represents the rotation from body to ECI (math/quaternion.hpp's
// convention, CLAUDE.md §6).
export type QuatCoeffs = [number, number, number, number];

export interface StateFrame {
  simTime: number;         // seconds since epoch
  // True trajectory
  truePos: [number, number, number];
  trueVel: [number, number, number];
  trueQuat: QuatCoeffs;        // Phase 5
  trueOmega: [number, number, number];  // Phase 5, body-frame rad/s
  // KF — unaffected by Phase 5: no attitude fields at all (§6.1), KF
  // doesn't carry an attitude state, so there is nothing to read here.
  kfPos: [number, number, number];
  kfVel: [number, number, number];
  kfCovDiag: [number, number, number, number, number, number];
  kfNis: number;
  // EKF — Phase 5 grows this from 6-state to 12-state MEKF.
  ekfPos: [number, number, number];
  ekfVel: [number, number, number];
  ekfQuat: QuatCoeffs;         // Phase 5: q_ref AFTER the MEKF reset — the actual attitude estimate
  ekfOmega: [number, number, number];   // Phase 5
  ekfCovDiag: [number, number, number, number, number, number, number, number, number, number, number, number]; // Phase 5: grown 6->12, [delta_theta(3),omega(3),r(3),v(3)]
  ekfNis: number;
  // UKF — same Phase 5 growth as EKF.
  ukfPos: [number, number, number];
  ukfVel: [number, number, number];
  ukfQuat: QuatCoeffs;
  ukfOmega: [number, number, number];
  ukfCovDiag: [number, number, number, number, number, number, number, number, number, number, number, number];
  ukfNis: number;
  // Fault
  activeFault: number;     // FaultType enum value, 0 = none
}

export const enum FaultType {
  None        = 0,
  GpsSpike    = 1,
  GpsDropout  = 2,
  Maneuver    = 3,
  DragError   = 4,
  SensorBias  = 5,
}

export interface FaultConfig {
  type: FaultType;
  onsetT: number;    // seconds
  duration: number;  // seconds (0 = instantaneous)
  magnitude: number; // fault-specific parameter
}

// Mirrors orbitforge::monte_carlo::FilterKind's declaration order
// (mc_runner.hpp) exactly — that order is the wire format (passed as a
// plain int across the ccall boundary), not just a convenient enum.
export const enum MCFilterKind {
  Kf  = 0,
  Ekf = 1,
  Ukf = 2,
}

// User-facing knobs for one Monte Carlo campaign (mc_results.ts). seed
// of -1 means "fresh every call" (the extern "C" run_monte_carlo wrapper,
// wasm_api.cpp, draws a real std::random_device value) — engine-internal
// MCConfig has no such sentinel, this is purely a JS/WASM-boundary
// convention, same as ScenarioConfig.seed's existing -1 = random.
export interface MCRunParams {
  nRuns: number;
  seed: number;
  filter: MCFilterKind;
  nSteps: number;
  dt: number;     // seconds between filter steps
  qPos: number;   // process noise std dev, position, meters
  qVel: number;   // process noise std dev, velocity, m/s
}

export interface MCStats {
  rmsPosPerStep: number[];
  rmsVelPerStep: number[];
  neesPerStep: number[];
  nisPerStep: number[];
  finalPosErrPerRun: number[]; // |r_true - r_hat| at the last step, one entry per run, meters
  // 95% consistency bounds for this campaign's n_runs — unlike
  // panels.ts's single-run NIS bounds, these genuinely depend on n_runs
  // (chi2(6*n_runs)/n_runs and chi2(3*n_runs)/n_runs).
  neesLower: number;
  neesUpper: number;
  nisLower: number;
  nisUpper: number;
}

// WASM module interface — populated by Emscripten glue after load
export interface OrbitForgeModule {
  ccall: (name: string, returnType: string | null, argTypes: string[], args: unknown[]) => unknown;
  cwrap: (name: string, returnType: string | null, argTypes: string[]) => (...args: unknown[]) => unknown;
  getValue: (ptr: number, type: string) => number;
  setValue: (ptr: number, value: number, type: string) => void;
  HEAPF64: Float64Array;
  HEAPU8: Uint8Array;
}
