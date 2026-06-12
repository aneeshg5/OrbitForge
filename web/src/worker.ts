// Web Worker: drives the 100 Hz simulation loop, owns the WASM module instance.
// Receives ScenarioConfig from main thread, writes StateFrames to SharedArrayBuffer ring buffer.

import type { ScenarioConfig, FaultConfig } from './bridge/wasm_types.js'

// TODO (Phase 2): import and initialize Emscripten WASM module
// TODO (Phase 2): implement 100 Hz sim loop using setInterval or Atomics.waitAsync

self.addEventListener('message', (e: MessageEvent) => {
  const { type, payload } = e.data as { type: string; payload: unknown }

  switch (type) {
    case 'init':
      // payload: ScenarioConfig
      console.log('[worker] init received, WASM not yet available')
      break
    case 'start':
      console.log('[worker] start received')
      break
    case 'pause':
      break
    case 'reset':
      break
    case 'set_fault':
      // payload: FaultConfig
      break
    default:
      console.warn('[worker] unknown message type:', type)
  }
})
