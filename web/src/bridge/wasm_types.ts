export interface ScenarioConfig {
  tleLine1: string;
  tleLine2: string;
  gpsSigma: number;
  imuSigma: number;
  enableJ2: boolean;
  enableDrag: boolean;
  enableSrp: boolean;
  dragCoeff: number;
  areaToMass: number;
  qPos: number;
  qVel: number;
  simSpeed: number;
  seed: number;
  inertiaX: number;
  inertiaY: number;
  inertiaZ: number;
  gyroSigma: number;
  magSigma: number;
  qAtt: number;
  qOmega: number;
  initOmegaX: number;
  initOmegaY: number;
  initOmegaZ: number;
}

export type QuatCoeffs = [number, number, number, number];

export interface StateFrame {
  simTime: number;
  truePos: [number, number, number];
  trueVel: [number, number, number];
  trueQuat: QuatCoeffs;
  trueOmega: [number, number, number];
  kfPos: [number, number, number];
  kfVel: [number, number, number];
  kfCovDiag: [number, number, number, number, number, number];
  kfNis: number;
  ekfPos: [number, number, number];
  ekfVel: [number, number, number];
  ekfQuat: QuatCoeffs;
  ekfOmega: [number, number, number];
  ekfCovDiag: [number, number, number, number, number, number, number, number, number, number, number, number];
  ekfNis: number;
  ukfPos: [number, number, number];
  ukfVel: [number, number, number];
  ukfQuat: QuatCoeffs;
  ukfOmega: [number, number, number];
  ukfCovDiag: [number, number, number, number, number, number, number, number, number, number, number, number];
  ukfNis: number;
  activeFault: number;
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
  onsetT: number;
  duration: number;
  magnitude: number;
}

export const enum MCFilterKind {
  Kf  = 0,
  Ekf = 1,
  Ukf = 2,
}

export interface MCRunParams {
  nRuns: number;
  seed: number;
  filter: MCFilterKind;
  nSteps: number;
  dt: number;
  qPos: number;
  qVel: number;
}

export interface MCStats {
  rmsPosPerStep: number[];
  rmsVelPerStep: number[];
  neesPerStep: number[];
  nisPerStep: number[];
  finalPosErrPerRun: number[];
  neesLower: number;
  neesUpper: number;
  nisLower: number;
  nisUpper: number;
}

export interface OrbitForgeModule {
  ccall: (name: string, returnType: string | null, argTypes: string[], args: unknown[]) => unknown;
  cwrap: (name: string, returnType: string | null, argTypes: string[]) => (...args: unknown[]) => unknown;
  getValue: (ptr: number, type: string) => number;
  setValue: (ptr: number, value: number, type: string) => void;
  HEAPF64: Float64Array;
  HEAPU8: Uint8Array;
}
