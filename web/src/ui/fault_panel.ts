// Fault injection controls: one button per fault type, each with an info
// button explaining what it actually does (mechanism, magnitude, and
// whether it's one-shot or persists for a window) so a user doesn't have
// to guess from the label alone.

import { FaultType, type FaultConfig } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'
import { makeInfoButton } from './info_button.js'

export interface FaultPanelOptions {
  postToWorker: (msg: WorkerRequest) => void
  // onset_t must be the CURRENT simulated time, not a fixed 0 — the engine
  // (engine/src/wasm_api.cpp's Simulation::step()) gates windowed faults
  // (GPS Dropout, GPS Bias) on `t_now < onset_t + duration`. A hardcoded
  // onset_t=0 makes that window "the first `duration` seconds since
  // Reset," not "duration seconds from when you clicked the button" — it
  // silently does nothing once more than `duration` sim-seconds have
  // elapsed since Reset, which in normal usage (run for a while, then
  // inject a fault to see how an already-converged filter responds) is
  // essentially always. One-shot faults (GPS Spike, Maneuver) don't use
  // onset_t in their gating at all, so this is harmless for them.
  getCurrentSimTimeSec: () => number
}

interface FaultButtonSpec {
  label: string
  type: FaultType
  duration: number
  magnitude: number
  info: string
}

const FAULT_BUTTONS: FaultButtonSpec[] = [
  {
    label: 'GPS Spike',
    type: FaultType.GpsSpike,
    duration: 0,
    magnitude: 500,
    info: 'Injects one bad GPS reading, 500m off, then returns to normal immediately. Tests how each filter rejects a single outlier.',
  },
  {
    label: 'GPS Dropout (60s)',
    type: FaultType.GpsDropout,
    duration: 60,
    magnitude: 0,
    info: 'Suppresses GPS entirely for 60 seconds starting now. The filters predict with no measurement to correct them, so their uncertainty grows until GPS resumes.',
  },
  {
    label: 'Maneuver (5 m/s)',
    type: FaultType.Maneuver,
    duration: 0,
    magnitude: 5,
    info: 'Gives the true spacecraft an instant 5 m/s velocity kick that the filters never see directly. Shows up as a sudden, unexplained drift in their position error over the following seconds.',
  },
  {
    label: 'Drag Coeff Error (+50%)',
    type: FaultType.DragError,
    duration: 0,
    magnitude: 0.5,
    info: 'Permanently raises the true drag coefficient by 50%, starting now, for the rest of this run (not a brief event). The filters keep assuming the original value, so their estimate slowly drifts as the real orbit decays faster than expected.',
  },
  {
    label: 'GPS Bias (+150m, 60s)',
    type: FaultType.SensorBias,
    duration: 60,
    magnitude: 150,
    info: 'Adds a constant 150m offset to every GPS reading for 60 seconds starting now, then removes it. Unlike GPS Spike (one bad reading) or GPS Dropout (no signal), this is a steady miscalibration the filters partially absorb, then have to recover from once it clears.',
  },
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
      const row = el('div', 'fault-row')
      const button = el('button', 'fault-btn')
      button.textContent = spec.label
      button.addEventListener('click', () => {
        const cfg: FaultConfig = {
          type: spec.type,
          onsetT: options.getCurrentSimTimeSec(),
          duration: spec.duration,
          magnitude: spec.magnitude,
        }
        options.postToWorker({ type: 'set_fault', payload: cfg })
      })
      row.append(button, makeInfoButton(spec.info))
      root.appendChild(row)
    }
    container.appendChild(root)
  }
}
