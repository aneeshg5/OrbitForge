// Entry point: spawn the simulation worker and start the 60 fps ring-buffer
// poll loop. Phase 3 wires in the WebGL2 Earth renderer, Chart.js panels,
// and the scenario editor that actually lets a user pick a satellite and
// send an 'init' message — until then this proves the worker/SharedArrayBuffer
// wiring described in CLAUDE.md §4 works end to end.

import { RingReader } from './bridge/ring_reader.js'
import type { WorkerRequest, WorkerResponse } from './worker.js'

let ringReader: RingReader | undefined

function startPollingLoop(): void {
  function tick(): void {
    if (ringReader) {
      // TODO (Phase 3): hand these frames to the WebGL renderer / Chart.js panels.
      ringReader.drain()
    }
    requestAnimationFrame(tick)
  }
  requestAnimationFrame(tick)
}

function registerServiceWorker(): void {
  if (!('serviceWorker' in navigator)) return
  navigator.serviceWorker.register('/sw.js').catch((err: unknown) => {
    console.warn('Service worker registration failed:', err)
  })
}

async function main(): Promise<void> {
  const app = document.getElementById('app')!
  app.textContent = 'OrbitForge — loading...'

  registerServiceWorker()

  const worker = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' })
  worker.addEventListener('message', (e: MessageEvent<WorkerResponse>) => {
    if (e.data.type === 'ring_buffer_ready') {
      const { sharedArrayBuffer, ringBufferPtr, ringBufferCapacity } = e.data.payload
      ringReader = new RingReader(sharedArrayBuffer, ringBufferPtr, ringBufferCapacity)
      startPollingLoop()
    }
  })

  // TODO (Phase 3): scenario_editor.ts posts the first WorkerRequest (type:
  // 'init') once the user picks a satellite and presses Run. Until then,
  // this helper exists so the worker wiring above has a typed call site.
  const postToWorker = (msg: WorkerRequest): void => worker.postMessage(msg)
  void postToWorker

  app.textContent = 'OrbitForge — worker ready, waiting for a scenario (Phase 3 UI not yet built).'
}

main().catch(err => console.error('OrbitForge init failed:', err))
