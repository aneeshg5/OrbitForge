// Entry point: load WASM module, spawn worker, initialize UI.
// Phase 3 will wire in the actual WebGL renderer and Chart.js panels.

import type { ScenarioConfig } from './bridge/wasm_types.js'

async function main(): Promise<void> {
  const app = document.getElementById('app')!
  app.textContent = 'OrbitForge — loading...'

  // TODO (Phase 2): load WASM module via dynamic import of Emscripten glue
  // TODO (Phase 2): spawn worker.ts, pass SharedArrayBuffer pointer
  // TODO (Phase 3): initialize WebGL2 earth renderer
  // TODO (Phase 3): initialize Chart.js panels

  app.textContent = 'OrbitForge — scaffold ready. WASM engine not yet compiled.'
}

main().catch(err => console.error('OrbitForge init failed:', err))
