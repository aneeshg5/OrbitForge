// Scenario editor: satellite picker (CelesTrak presets or pasted TLE),
// GPS sigma / sim speed sliders, and perturbation toggles. Exposes
// getConfig() so the topbar RunControls can read the current scenario
// without this editor needing to know about run/pause/reset.

import { PRESETS, fetchTleByNorad } from '../data/tle_feed.js'
import type { ScenarioConfig } from '../bridge/wasm_types.js'
import { showToast } from './toast.js'

export interface ScenarioEditorOptions {
  // Fired whenever whether getConfig() would currently succeed changes —
  // lets RunControls disable the Run button instead of silently no-op'ing
  // when clicked with no TLE loaded yet (the bug this was added to fix).
  onAvailabilityChange?: (available: boolean) => void
}

// Sim speed multiplies the RK4 integrator's per-tick step directly
// (engine/src/wasm_api.cpp: dt = (1/100Hz) * sim_speed), so it isn't just a
// playback-rate knob — it's literally the physics step size. At 1000x, dt
// is already 10s/step (~0.2% of the ISS preset's ~5500s orbital period);
// well beyond a few thousand x, step size becomes a non-trivial fraction
// of a LEO period (worst-case the eccentric debris preset's fast perigee
// passage) and the RK4 "true" trajectory itself starts accumulating
// visible integration error, not just looking sped-up. 5000 is a generous
// ceiling that stays in the regime validated by this engine's existing
// energy-conservation tests (h~10s fixed-step, CLAUDE.md §6 Integrators),
// not a precisely re-derived bound — happy to actually sweep this
// empirically against the conservation tests if finer-grained accuracy
// matters for a specific scenario.
const SIM_SPEED_MAX = 5000

const DEFAULTS = {
  gpsSigma: 10,
  // 86400 (seconds/sim-day) / 20 (seconds for the Earth's cosmetic spin to
  // complete one rotation at this speed, main.ts's SPIN_RAD_PER_SIM_SECOND)
  // — keeps the same ~20s/rotation pacing the old wall-clock-driven spin
  // had, now achieved by actually running sim_time fast enough rather than
  // decoupling the visual from sim_time. Below SIM_SPEED_MAX, comfortably
  // inside the dt-vs-orbital-period regime that comment justifies.
  simSpeed: 4320,
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
  // Bumped on every onSatelliteChange() call; lets an in-flight fetch
  // detect it's been superseded by a newer selection and bail out instead
  // of overwriting currentTle/statusLine/availability with stale results.
  // Without this, switching satellites while a slow CelesTrak request is
  // still in flight (the auto-fetch on page load is itself one such
  // request) could have an old fetch resolve after a newer one and leave
  // the Run button disabled (or showing the wrong "Loaded X") even though
  // the satellite you're actually looking at loaded fine.
  private requestSeq = 0

  private readonly satelliteSelect: HTMLSelectElement
  private readonly tleTextarea: HTMLTextAreaElement
  private readonly gpsSigmaInput: HTMLInputElement
  private readonly gpsSigmaLabel: HTMLSpanElement
  private readonly simSpeedInput: HTMLInputElement
  private readonly simSpeedNumberInput: HTMLInputElement
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
    // Only relevant when "(paste TLE below)" is selected — hidden the rest
    // of the time so the panel isn't cluttered with an inert text box
    // while a preset satellite is active.
    this.tleTextarea.style.display = 'none'

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
    // Range covers the common 0.1x-100x sweep with a quick drag gesture;
    // the paired number input (synced to the same value) accepts anything
    // up to SIM_SPEED_MAX for users who want a precise or larger value the
    // slider's range can't reach.
    this.simSpeedInput = el('input')
    this.simSpeedInput.type = 'range'
    this.simSpeedInput.min = '0.1'
    this.simSpeedInput.max = '100'
    this.simSpeedInput.step = '0.1'
    this.simSpeedInput.value = String(DEFAULTS.simSpeed)
    this.simSpeedNumberInput = el('input')
    this.simSpeedNumberInput.type = 'number'
    this.simSpeedNumberInput.className = 'sim-speed-number'
    this.simSpeedNumberInput.min = '0.1'
    this.simSpeedNumberInput.max = String(SIM_SPEED_MAX)
    this.simSpeedNumberInput.step = '0.1'
    this.simSpeedNumberInput.value = String(DEFAULTS.simSpeed)
    const speedUnit = el('span')
    speedUnit.textContent = 'x'

    this.simSpeedInput.addEventListener('input', () => {
      this.simSpeedNumberInput.value = this.simSpeedInput.value
    })
    this.simSpeedNumberInput.addEventListener('input', () => {
      const value = Number(this.simSpeedNumberInput.value)
      if (!Number.isFinite(value) || value <= 0) return
      const clamped = Math.min(value, SIM_SPEED_MAX)
      // Only mirror onto the slider when it's within the slider's own
      // narrower range — typing e.g. 2000 should leave the slider pinned
      // at its max rather than silently snapping the typed value down.
      if (clamped >= 0.1 && clamped <= 100) this.simSpeedInput.value = String(clamped)
    })
    this.simSpeedNumberInput.addEventListener('change', () => {
      const value = Number(this.simSpeedNumberInput.value)
      const clamped = !Number.isFinite(value) || value <= 0 ? DEFAULTS.simSpeed : Math.min(value, SIM_SPEED_MAX)
      // Snap to the 0.1 step on commit (blur/enter), not on every
      // keystroke — rounding mid-type would fight whatever the user is
      // currently typing (e.g. "3." while reaching for "3.5").
      const rounded = Math.round(clamped * 10) / 10
      this.simSpeedNumberInput.value = String(rounded)
      if (rounded <= 100) this.simSpeedInput.value = String(Math.max(rounded, 0.1))
    })
    speedRow.append(speedLabel, this.simSpeedInput, this.simSpeedNumberInput, speedUnit)

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

  // Plain text status (or cleared) — also drops the loading spinner if one
  // was showing.
  private setStatusText(text: string): void {
    this.statusLine.classList.remove('loading')
    this.statusLine.textContent = text
  }

  // In-flight fetch indicator: a small spinner + "Loading <label>…", built
  // via DOM API rather than innerHTML (this codebase's convention — see
  // run_controls.ts's setButtonContent). The spinner itself is pure CSS
  // (index.html's .status-line.loading .spinner), not an image/icon font.
  private setStatusLoading(label: string): void {
    this.statusLine.classList.add('loading')
    const spinner = el('span', 'spinner')
    this.statusLine.replaceChildren(spinner, document.createTextNode(`Loading ${label}…`))
  }

  private async onSatelliteChange(): Promise<void> {
    const seq = ++this.requestSeq
    const value = this.satelliteSelect.value
    this.tleTextarea.style.display = value === '__paste__' ? '' : 'none'

    if (value === '__paste__') {
      this.currentTle = undefined
      this.setStatusText('')
      this.notifyAvailability()
      return
    }
    const noradId = Number(value)
    this.currentTle = undefined
    this.notifyAvailability()
    const presetLabel = PRESETS.find((p) => p.noradId === noradId)?.name ?? `NORAD ${noradId}`
    this.setStatusLoading(presetLabel)
    try {
      const result = await fetchTleByNorad(noradId)
      if (seq !== this.requestSeq) return // superseded by a newer selection — discard this result
      const elements = result.elements
      this.currentTle = { line1: elements.tleLine1, line2: elements.tleLine2 }
      const label = elements.name || `NORAD ${noradId}`
      if (result.fromCache) {
        this.setStatusText(`Loaded ${label} (cached, offline)`)
        showToast(`CelesTrak unreachable — using cached TLE for ${label} (cached ${result.cachedAt}, may be stale)`, 'info')
      } else {
        this.setStatusText(`Loaded ${label}`)
      }
    } catch (err) {
      if (seq !== this.requestSeq) return // superseded — don't disable Run for a selection the user already changed away from
      this.setStatusText('')
      showToast(`Failed to fetch TLE for NORAD ${noradId}: ${String(err)}`, 'error')
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
  // why via a toast) if no TLE is available yet.
  getConfig(): ScenarioConfig | undefined {
    const tle = this.satelliteSelect.value === '__paste__' ? this.parsePastedTle() : this.currentTle
    if (!tle) {
      showToast('No valid TLE selected or pasted.', 'error')
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
      simSpeed: Number(this.simSpeedNumberInput.value),
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
