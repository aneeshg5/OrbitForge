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
}

export interface StateFrame {
  simTime: number;         // seconds since epoch
  // True trajectory
  truePos: [number, number, number];
  trueVel: [number, number, number];
  // KF
  kfPos: [number, number, number];
  kfVel: [number, number, number];
  kfCovDiag: [number, number, number, number, number, number];
  kfNis: number;
  // EKF
  ekfPos: [number, number, number];
  ekfVel: [number, number, number];
  ekfCovDiag: [number, number, number, number, number, number];
  ekfNis: number;
  // UKF
  ukfPos: [number, number, number];
  ukfVel: [number, number, number];
  ukfCovDiag: [number, number, number, number, number, number];
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
