// Fault injection controls: one button per fault type.
// onset_t is always 0 — wasm_api.cpp's Simulation::step() applies a fault
// once `t_now >= active_fault_.onset_t`, and since sim_time only increases
// from 0, onset_t=0 always means "apply on the next tick" regardless of
// how far the simulation has already run, with no need to track or read
// the current sim time from the UI layer at all.

import { FaultType, type FaultConfig } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'

export interface FaultPanelOptions {
  postToWorker: (msg: WorkerRequest) => void
}

interface FaultButtonSpec {
  label: string
  type: FaultType
  duration: number
  magnitude: number
}

// Default magnitudes per fault type.
const FAULT_BUTTONS: FaultButtonSpec[] = [
  { label: 'GPS Spike', type: FaultType.GpsSpike, duration: 0, magnitude: 500 },
  { label: 'GPS Dropout (60s)', type: FaultType.GpsDropout, duration: 60, magnitude: 0 },
  { label: 'Maneuver (5 m/s)', type: FaultType.Maneuver, duration: 0, magnitude: 5 },
  { label: 'Drag Coeff Error (+50%)', type: FaultType.DragError, duration: 0, magnitude: 0.5 },
  { label: 'Sensor Bias', type: FaultType.SensorBias, duration: 0, magnitude: 0.02 },
]

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

export class FaultPanel {
  constructor(container: HTMLElement, options: FaultPanelOptions) {
    const heading = el('h3')
    heading.textContent = 'Fault Injection'
    container.appendChild(heading)

    const root = el('div', 'fault-panel')
    for (const spec of FAULT_BUTTONS) {
      const button = el('button')
      button.textContent = spec.label
      button.addEventListener('click', () => {
        const cfg: FaultConfig = {
          type: spec.type,
          onsetT: 0,
          duration: spec.duration,
          magnitude: spec.magnitude,
        }
        options.postToWorker({ type: 'set_fault', payload: cfg })
      })
      root.appendChild(button)
    }
    container.appendChild(root)
  }
}
