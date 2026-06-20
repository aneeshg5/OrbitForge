// Scenario editor: satellite picker (CelesTrak presets or pasted TLE),
// GPS sigma / sim speed sliders, and perturbation toggles. Exposes
// getConfig() so the topbar RunControls can read the current scenario
// without this editor needing to know about run/pause/reset.

import { PRESETS, fetchTleByNorad } from '../data/tle_feed.js'
import type { OrbitalElements } from '../data/tle_parser.js'
import type { ScenarioConfig } from '../bridge/wasm_types.js'

export interface ScenarioEditorOptions {
  // Fired whenever whether getConfig() would currently succeed changes —
  // lets RunControls disable the Run button instead of silently no-op'ing
  // when clicked with no TLE loaded yet (the bug this was added to fix).
  onAvailabilityChange?: (available: boolean) => void
}

const DEFAULTS = {
  gpsSigma: 10,
  simSpeed: 1,
  dragCoeff: 2.2,
  areaToMass: 0.01,
  qPos: 1.0,
  qVel: 0.01,
  imuSigma: 0.05,
  // Phase 5: 6DOF — no UI controls for these yet (CLAUDE.md §21), fixed
  // defaults matching engine/include/scenario.hpp's ScenarioCfg defaults.
  inertia: 1.0,
  gyroSigma: 0.001,
  magSigma: 100.0,
  qAtt: 1e-6,
  qOmega: 1e-8,
  initOmegaZ: 0.05,
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

export class ScenarioEditor {
  private readonly root: HTMLElement
  private currentTle: { line1: string; line2: string } | undefined

  private readonly satelliteSelect: HTMLSelectElement
  private readonly tleTextarea: HTMLTextAreaElement
  private readonly gpsSigmaInput: HTMLInputElement
  private readonly gpsSigmaLabel: HTMLSpanElement
  private readonly simSpeedInput: HTMLInputElement
  private readonly simSpeedLabel: HTMLSpanElement
  private readonly j2Checkbox: HTMLInputElement
  private readonly dragCheckbox: HTMLInputElement
  private readonly srpCheckbox: HTMLInputElement
  private readonly statusLine: HTMLDivElement
  private readonly onAvailabilityChange: (available: boolean) => void

  constructor(container: HTMLElement, options: ScenarioEditorOptions = {}) {
    this.onAvailabilityChange = options.onAvailabilityChange ?? (() => {})
    this.root = el('div', 'scenario-editor')

    const heading = el('h3')
    heading.textContent = 'Scenario'
    this.root.appendChild(heading)

    const satelliteRow = el('div', 'row')
    const satelliteLabel = el('label')
    satelliteLabel.textContent = 'Satellite: '
    this.satelliteSelect = el('select')
    const pasteOption = el('option')
    pasteOption.value = '__paste__'
    pasteOption.textContent = '(paste TLE below)'
    this.satelliteSelect.appendChild(pasteOption)
    for (const preset of PRESETS) {
      const opt = el('option')
      opt.value = String(preset.noradId)
      opt.textContent = `${preset.name} — ${preset.whyInteresting}`
      this.satelliteSelect.appendChild(opt)
    }
    satelliteRow.append(satelliteLabel, this.satelliteSelect)

    this.tleTextarea = el('textarea')
    this.tleTextarea.placeholder = 'Paste a 2-line TLE here if not selecting a preset above'
    this.tleTextarea.rows = 2

    const gpsRow = el('div', 'row')
    const gpsLabel = el('label')
    gpsLabel.textContent = 'GPS σ: '
    this.gpsSigmaInput = el('input')
    this.gpsSigmaInput.type = 'range'
    this.gpsSigmaInput.min = '1'
    this.gpsSigmaInput.max = '100'
    this.gpsSigmaInput.value = String(DEFAULTS.gpsSigma)
    this.gpsSigmaLabel = el('span')
    this.gpsSigmaLabel.textContent = `${DEFAULTS.gpsSigma} m`
    this.gpsSigmaInput.addEventListener('input', () => {
      this.gpsSigmaLabel.textContent = `${this.gpsSigmaInput.value} m`
    })
    gpsRow.append(gpsLabel, this.gpsSigmaInput, this.gpsSigmaLabel)

    const speedRow = el('div', 'row')
    const speedLabel = el('label')
    speedLabel.textContent = 'Sim speed: '
    this.simSpeedInput = el('input')
    this.simSpeedInput.type = 'range'
    this.simSpeedInput.min = '1'
    this.simSpeedInput.max = '100'
    this.simSpeedInput.value = String(DEFAULTS.simSpeed)
    this.simSpeedLabel = el('span')
    this.simSpeedLabel.textContent = `${DEFAULTS.simSpeed}x`
    this.simSpeedInput.addEventListener('input', () => {
      this.simSpeedLabel.textContent = `${this.simSpeedInput.value}x`
    })
    speedRow.append(speedLabel, this.simSpeedInput, this.simSpeedLabel)

    const j2Field = this.makeCheckbox('J2', true)
    const dragField = this.makeCheckbox('Drag', true)
    const srpField = this.makeCheckbox('SRP', false)
    this.j2Checkbox = j2Field.checkbox
    this.dragCheckbox = dragField.checkbox
    this.srpCheckbox = srpField.checkbox

    const perturbRow = el('div', 'row')
    perturbRow.append(j2Field.label, dragField.label, srpField.label)

    this.statusLine = el('div', 'status-line')

    this.root.append(satelliteRow, this.tleTextarea, gpsRow, speedRow, perturbRow, this.statusLine)
    container.appendChild(this.root)

    this.satelliteSelect.addEventListener('change', () => {
      void this.onSatelliteChange()
    })
    this.tleTextarea.addEventListener('input', () => {
      this.notifyAvailability()
    })

    // Disabled until the first fetch (or paste) resolves — see
    // notifyAvailability(); avoids the Run button silently no-op'ing.
    this.notifyAvailability()

    // Default to the first preset (ISS) so Run works without extra clicks.
    if (PRESETS.length > 0) {
      this.satelliteSelect.value = String(PRESETS[0]!.noradId)
      void this.onSatelliteChange()
    }
  }

  private hasValidTle(): boolean {
    return this.satelliteSelect.value === '__paste__' ? this.parsePastedTle() !== undefined : this.currentTle !== undefined
  }

  private notifyAvailability(): void {
    this.onAvailabilityChange(this.hasValidTle())
  }

  private makeCheckbox(name: string, checked: boolean): { label: HTMLLabelElement; checkbox: HTMLInputElement } {
    const label = el('label')
    const checkbox = el('input')
    checkbox.type = 'checkbox'
    checkbox.checked = checked
    label.append(checkbox, document.createTextNode(` ${name}`))
    return { label, checkbox }
  }

  private async onSatelliteChange(): Promise<void> {
    const value = this.satelliteSelect.value
    if (value === '__paste__') {
      this.currentTle = undefined
      this.notifyAvailability()
      return
    }
    const noradId = Number(value)
    this.currentTle = undefined
    this.notifyAvailability()
    try {
      const elements: OrbitalElements = await fetchTleByNorad(noradId)
      this.currentTle = { line1: elements.tleLine1, line2: elements.tleLine2 }
      this.statusLine.textContent = `Loaded ${elements.name || `NORAD ${noradId}`}`
    } catch (err) {
      this.statusLine.textContent = `Failed to fetch TLE: ${String(err)}`
      this.currentTle = undefined
    }
    this.notifyAvailability()
  }

  private parsePastedTle(): { line1: string; line2: string } | undefined {
    const lines = this.tleTextarea.value
      .split('\n')
      .map((l) => l.trim())
      .filter((l) => l.length > 0)
    if (lines.length < 2) return undefined
    const line1 = lines.find((l) => l.startsWith('1 '))
    const line2 = lines.find((l) => l.startsWith('2 '))
    if (!line1 || !line2) return undefined
    return { line1, line2 }
  }

  // Returns the scenario config to launch with, or undefined (and reports
  // why via the status line) if no TLE is available yet.
  getConfig(): ScenarioConfig | undefined {
    const tle = this.satelliteSelect.value === '__paste__' ? this.parsePastedTle() : this.currentTle
    if (!tle) {
      this.statusLine.textContent = 'No valid TLE selected or pasted.'
      return undefined
    }

    return {
      tleLine1: tle.line1,
      tleLine2: tle.line2,
      gpsSigma: Number(this.gpsSigmaInput.value),
      imuSigma: DEFAULTS.imuSigma,
      enableJ2: this.j2Checkbox.checked,
      enableDrag: this.dragCheckbox.checked,
      enableSrp: this.srpCheckbox.checked,
      dragCoeff: DEFAULTS.dragCoeff,
      areaToMass: DEFAULTS.areaToMass,
      qPos: DEFAULTS.qPos,
      qVel: DEFAULTS.qVel,
      simSpeed: Number(this.simSpeedInput.value),
      seed: -1,
      inertiaX: DEFAULTS.inertia,
      inertiaY: DEFAULTS.inertia,
      inertiaZ: DEFAULTS.inertia,
      gyroSigma: DEFAULTS.gyroSigma,
      magSigma: DEFAULTS.magSigma,
      qAtt: DEFAULTS.qAtt,
      qOmega: DEFAULTS.qOmega,
      initOmegaX: 0,
      initOmegaY: 0,
      initOmegaZ: DEFAULTS.initOmegaZ,
    }
  }
}
