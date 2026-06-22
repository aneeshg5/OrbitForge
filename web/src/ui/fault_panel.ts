// Fault injection controls: one button per fault type, each with editable
// value(s) for whatever that fault actually uses (verified against the
// engine's switch-case in engine/src/wasm_api.cpp's Simulation::step() —
// only the fields a given fault's case branch reads are exposed, so there's
// nothing to edit that silently does nothing) and an info button explaining
// the mechanism in plain language.

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

// One editable numeric field (magnitude or duration), in whatever unit
// reads best for that fault — `toRaw` converts the typed UI value into
// the unit FaultConfig actually expects (only Drag Coeff Error needs this,
// for percent-to-fraction).
interface FieldSpec {
  unit: string
  default: number
  step: number
  min?: number
  toRaw?: (uiValue: number) => number
}

interface FaultButtonSpec {
  label: string
  type: FaultType
  info: string
  magnitude?: FieldSpec
  duration?: FieldSpec
}

const FAULT_BUTTONS: FaultButtonSpec[] = [
  {
    label: 'GPS Spike',
    type: FaultType.GpsSpike,
    magnitude: { unit: 'm', default: 500, step: 10 },
    info: 'Injects one bad GPS reading, offset by the given amount, then returns to normal immediately. Tests how each filter rejects a single outlier.',
  },
  {
    label: 'GPS Dropout',
    type: FaultType.GpsDropout,
    duration: { unit: 's', default: 60, step: 5, min: 1 },
    info: 'Suppresses GPS entirely for the given duration starting now. The filters predict with no measurement to correct them, so their uncertainty grows until GPS resumes.',
  },
  {
    label: 'Maneuver',
    type: FaultType.Maneuver,
    magnitude: { unit: 'm/s', default: 5, step: 1 },
    info: 'Gives the true spacecraft an instant velocity kick (prograde for positive values, retrograde for negative) that the filters never see directly. Shows up as a sudden, unexplained drift in their position error over the following seconds.',
  },
  {
    label: 'Drag Coeff Error',
    type: FaultType.DragError,
    magnitude: { unit: '%', default: 50, step: 5, min: -99, toRaw: (pct) => pct / 100 },
    info: 'Permanently raises (or, for a negative value, lowers) the true drag coefficient by the given percentage, starting now, for the rest of this run — not a brief event. The filters keep assuming the original value, so their estimate slowly drifts as the real orbit decays faster or slower than expected.',
  },
  {
    label: 'GPS Bias',
    type: FaultType.SensorBias,
    magnitude: { unit: 'm', default: 150, step: 10 },
    duration: { unit: 's', default: 60, step: 5, min: 1 },
    info: 'Adds a constant offset to every GPS reading for the given duration starting now, then removes it. Unlike GPS Spike (one bad reading) or GPS Dropout (no signal), this is a steady miscalibration the filters partially absorb, then have to recover from once it clears.',
  },
]

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

// Builds a labeled number input for one FieldSpec, returning the element to
// mount plus a getter for its current value already converted via toRaw.
function makeField(spec: FieldSpec): { el: HTMLElement; getValue: () => number } {
  const wrapper = el('span', 'fault-field')
  const input = el('input', 'fault-field-input')
  input.type = 'number'
  input.value = String(spec.default)
  input.step = String(spec.step)
  if (spec.min !== undefined) input.min = String(spec.min)
  const unit = el('span', 'fault-field-unit')
  unit.textContent = spec.unit
  wrapper.append(input, unit)
  return {
    el: wrapper,
    getValue: () => {
      const raw = Number(input.value)
      const value = Number.isFinite(raw) ? raw : spec.default
      return spec.toRaw ? spec.toRaw(value) : value
    },
  }
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

      const magnitudeField = spec.magnitude ? makeField(spec.magnitude) : undefined
      const durationField = spec.duration ? makeField(spec.duration) : undefined

      button.addEventListener('click', () => {
        const cfg: FaultConfig = {
          type: spec.type,
          onsetT: options.getCurrentSimTimeSec(),
          duration: durationField ? durationField.getValue() : 0,
          magnitude: magnitudeField ? magnitudeField.getValue() : 0,
        }
        options.postToWorker({ type: 'set_fault', payload: cfg })
      })

      row.append(button)
      if (magnitudeField) row.append(magnitudeField.el)
      if (durationField) row.append(durationField.el)
      row.append(makeInfoButton(spec.info))
      root.appendChild(row)
    }
    container.appendChild(root)
  }
}
