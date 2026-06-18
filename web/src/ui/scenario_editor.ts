// Scenario editor: satellite picker (CelesTrak presets or
// pasted TLE), GPS sigma / sim speed sliders, perturbation toggles, and
// Run/Pause/Reset controls. Posts WorkerRequest messages through the
// caller-supplied `postToWorker` callback — never calls ccall directly;
// all WASM calls go through bridge/wasm_types.ts.

import { PRESETS, fetchTleByNorad } from '../data/tle_feed.js'
import type { OrbitalElements } from '../data/tle_parser.js'
import type { ScenarioConfig } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'

export interface ScenarioEditorOptions {
  postToWorker: (msg: WorkerRequest) => void
}

const DEFAULTS = {
  gpsSigma: 10,
  simSpeed: 1,
  dragCoeff: 2.2,
  areaToMass: 0.01,
  qPos: 1.0,
  qVel: 0.01,
  imuSigma: 0.05,
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

export class ScenarioEditor {
  private readonly root: HTMLElement
  private readonly postToWorker: (msg: WorkerRequest) => void
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
  private readonly runButton: HTMLButtonElement
  private readonly statusLine: HTMLDivElement

  constructor(container: HTMLElement, options: ScenarioEditorOptions) {
    this.postToWorker = options.postToWorker
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

    const buttonRow = el('div', 'row')
    this.runButton = el('button')
    this.runButton.textContent = '▶ Run'
    const pauseButton = el('button')
    pauseButton.textContent = '⏸ Pause'
    const resetButton = el('button')
    resetButton.textContent = '⟳ Reset'
    buttonRow.append(this.runButton, pauseButton, resetButton)

    this.statusLine = el('div', 'status-line')

    this.root.append(satelliteRow, this.tleTextarea, gpsRow, speedRow, perturbRow, buttonRow, this.statusLine)
    container.appendChild(this.root)

    this.satelliteSelect.addEventListener('change', () => {
      void this.onSatelliteChange()
    })
    this.runButton.addEventListener('click', () => this.onRun())
    pauseButton.addEventListener('click', () => this.postToWorker({ type: 'pause' }))
    resetButton.addEventListener('click', () => this.postToWorker({ type: 'reset' }))

    // Default to the first preset (ISS) so Run works without extra clicks.
    if (PRESETS.length > 0) {
      this.satelliteSelect.value = String(PRESETS[0]!.noradId)
      void this.onSatelliteChange()
    }
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
      return
    }
    const noradId = Number(value)
    this.statusLine.textContent = 'Fetching TLE from CelesTrak...'
    try {
      const elements: OrbitalElements = await fetchTleByNorad(noradId)
      this.currentTle = { line1: elements.tleLine1, line2: elements.tleLine2 }
      this.statusLine.textContent = `Loaded ${elements.name || `NORAD ${noradId}`}`
    } catch (err) {
      this.statusLine.textContent = `Failed to fetch TLE: ${String(err)}`
      this.currentTle = undefined
    }
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

  private onRun(): void {
    const tle = this.satelliteSelect.value === '__paste__' ? this.parsePastedTle() : this.currentTle
    if (!tle) {
      this.statusLine.textContent = 'No valid TLE selected or pasted.'
      return
    }

    const cfg: ScenarioConfig = {
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
    }

    this.postToWorker({ type: 'init', payload: cfg })
    this.postToWorker({ type: 'start' })
    this.statusLine.textContent = 'Running.'
  }
}
