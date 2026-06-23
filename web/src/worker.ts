import type { FaultConfig, MCRunParams, MCStats, OrbitForgeModule, ScenarioConfig } from './bridge/wasm_types.js'

export interface InitMessage {
  type: 'init'
  payload: ScenarioConfig
}
export interface ControlMessage {
  type: 'start' | 'pause' | 'reset'
}
export interface SetFaultMessage {
  type: 'set_fault'
  payload: FaultConfig
}
// Only safe to send while paused/idle: set_sim_speed() is unsynchronized
// against run_loop()'s own thread.
export interface SetSimSpeedMessage {
  type: 'set_sim_speed'
  payload: { simSpeed: number }
}
export interface RunMonteCarloMessage {
  type: 'run_monte_carlo'
  payload: MCRunParams
}
export type WorkerRequest =
  | InitMessage
  | ControlMessage
  | SetFaultMessage
  | SetSimSpeedMessage
  | RunMonteCarloMessage

export interface RingBufferReadyMessage {
  type: 'ring_buffer_ready'
  payload: {
    sharedArrayBuffer: SharedArrayBuffer
    ringBufferPtr: number
    ringBufferCapacity: number
    mcProgressPtr: number
  }
}
export interface McResultsMessage {
  type: 'mc_results'
  payload: MCStats
}
export type WorkerResponse = RingBufferReadyMessage | McResultsMessage

type WasmModuleFactory = (overrides?: { locateFile?: (path: string) => string }) => Promise<OrbitForgeModule>

let wasmModulePromise: Promise<OrbitForgeModule> | undefined

async function loadWasmModule(): Promise<OrbitForgeModule> {
  const code = await (await fetch('/orbitforge.js')).text()
  const blobUrl = URL.createObjectURL(new Blob([code], { type: 'text/javascript' }))
  try {
    const imported = (await import(/* @vite-ignore */ blobUrl)) as { default: WasmModuleFactory }
    return imported.default({ locateFile: (path: string) => `/${path}` })
  } finally {
    URL.revokeObjectURL(blobUrl)
  }
}

function getWasmModule(): Promise<OrbitForgeModule> {
  if (!wasmModulePromise) wasmModulePromise = loadWasmModule()
  return wasmModulePromise
}

function initScenario(module: OrbitForgeModule, cfg: ScenarioConfig): void {
  module.ccall(
    'init_scenario',
    null,
    ['string', 'string', 'number', 'number', 'number', 'number', 'number',
     'number', 'number', 'number', 'number', 'number', 'number',
     'number', 'number', 'number', 'number', 'number', 'number', 'number',
     'number', 'number', 'number'],
    [cfg.tleLine1, cfg.tleLine2, cfg.gpsSigma, cfg.imuSigma,
     cfg.enableJ2 ? 1 : 0, cfg.enableDrag ? 1 : 0, cfg.enableSrp ? 1 : 0,
     cfg.dragCoeff, cfg.areaToMass, cfg.qPos, cfg.qVel, cfg.simSpeed, cfg.seed,
     cfg.inertiaX, cfg.inertiaY, cfg.inertiaZ, cfg.gyroSigma, cfg.magSigma,
     cfg.qAtt, cfg.qOmega, cfg.initOmegaX, cfg.initOmegaY, cfg.initOmegaZ],
  )

  const ringBufferPtr = module.ccall('get_ring_buffer_ptr', 'number', [], []) as number
  const ringBufferCapacity = module.ccall('get_ring_buffer_capacity', 'number', [], []) as number
  const mcProgressPtr = module.ccall('get_mc_progress_ptr', 'number', [], []) as number

  const response: RingBufferReadyMessage = {
    type: 'ring_buffer_ready',
    payload: {
      sharedArrayBuffer: module.HEAPF64.buffer as SharedArrayBuffer,
      ringBufferPtr,
      ringBufferCapacity,
      mcProgressPtr,
    },
  }
  self.postMessage(response)
}

function readF64Array(module: OrbitForgeModule, ptr: number, len: number): number[] {
  const base = ptr / 8
  const out = new Array<number>(len)
  for (let i = 0; i < len; i++) out[i] = module.HEAPF64[base + i]!
  return out
}

function runMonteCarlo(module: OrbitForgeModule, params: MCRunParams): MCStats {
  module.ccall(
    'run_monte_carlo',
    null,
    ['number', 'number', 'number', 'number', 'number', 'number', 'number'],
    [params.nRuns, params.seed, params.filter, params.nSteps, params.dt, params.qPos, params.qVel],
  )

  const nSteps = module.ccall('get_mc_n_steps', 'number', [], []) as number
  const nRunsActual = module.ccall('get_mc_n_runs', 'number', [], []) as number
  const rmsPosPtr = module.ccall('get_mc_rms_pos_ptr', 'number', [], []) as number
  const rmsVelPtr = module.ccall('get_mc_rms_vel_ptr', 'number', [], []) as number
  const neesPtr = module.ccall('get_mc_nees_ptr', 'number', [], []) as number
  const nisPtr = module.ccall('get_mc_nis_ptr', 'number', [], []) as number
  const finalPosErrPtr = module.ccall('get_mc_final_pos_err_ptr', 'number', [], []) as number

  return {
    rmsPosPerStep: readF64Array(module, rmsPosPtr, nSteps),
    rmsVelPerStep: readF64Array(module, rmsVelPtr, nSteps),
    neesPerStep: readF64Array(module, neesPtr, nSteps),
    nisPerStep: readF64Array(module, nisPtr, nSteps),
    finalPosErrPerRun: readF64Array(module, finalPosErrPtr, nRunsActual),
    neesLower: module.ccall('get_mc_nees_lower', 'number', [], []) as number,
    neesUpper: module.ccall('get_mc_nees_upper', 'number', [], []) as number,
    nisLower: module.ccall('get_mc_nis_lower', 'number', [], []) as number,
    nisUpper: module.ccall('get_mc_nis_upper', 'number', [], []) as number,
  }
}

self.addEventListener('message', (e: MessageEvent<WorkerRequest>) => {
  const msg = e.data
  void getWasmModule().then((module) => {
    switch (msg.type) {
      case 'init':
        initScenario(module, msg.payload)
        break
      case 'start':
        module.ccall('start_simulation', null, [], [])
        break
      case 'pause':
        module.ccall('pause_simulation', null, [], [])
        break
      case 'reset':
        module.ccall('reset_simulation', null, [], [])
        break
      case 'set_fault':
        module.ccall(
          'set_fault',
          null,
          ['number', 'number', 'number', 'number'],
          [msg.payload.type, msg.payload.onsetT, msg.payload.duration, msg.payload.magnitude],
        )
        break
      case 'set_sim_speed':
        module.ccall('set_sim_speed', null, ['number'], [msg.payload.simSpeed])
        break
      case 'run_monte_carlo': {
        const stats = runMonteCarlo(module, msg.payload)
        const response: McResultsMessage = { type: 'mc_results', payload: stats }
        self.postMessage(response)
        break
      }
    }
  })
})
