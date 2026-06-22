// Monte Carlo results panel: a small parameter form (runs, filter, run
// duration, process noise, seed) + "Run MC" button, plus three result
// widgets once a campaign completes — a final-position-error histogram, an
// RMS table, and NEES/NIS consistency charts with the campaign's own
// chi-squared bounds (not the fixed single-run bounds renderer/panels.ts
// uses). Posts a 'run_monte_carlo' WorkerRequest through the
// caller-supplied callback; never calls ccall directly.
//
// None of these fields had any UI before — n_runs was the only knob
// (a slider), everything else (filter, n_steps/dt, process noise, seed)
// was hardcoded in wasm_api.cpp's Simulation::run_monte_carlo(). Plain
// number inputs throughout, not sliders (matches scenario_editor.ts's GPS
// σ / sim speed precedent — a slider's fixed range can't usefully cover
// both "a quick 50-run check" and "a smooth 5000-run histogram," same
// reasoning that already ruled out a slider for sim speed).

import {
  Chart,
  BarController,
  BarElement,
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Legend,
  Tooltip,
} from 'chart.js'
import { MCFilterKind, type MCStats, type ScenarioConfig } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'
import { makeInfoButton } from './info_button.js'

Chart.register(
  BarController, BarElement, LineController, LineElement, PointElement,
  LinearScale, CategoryScale, Legend, Tooltip,
)

export interface MCResultsOptions {
  postToWorker: (msg: WorkerRequest) => void
  // Read on a Run MC click that hasn't been preceded by any init (see
  // onRunMC()) — lets Monte Carlo init the live Simulation itself instead
  // of requiring the user to click the topbar Run button first just to
  // populate x_true_initial_, which had no visible connection to "fill out
  // the MC fields and click Run MC" from the user's side.
  getConfig: () => ScenarioConfig | undefined
}

const TEXT_MUTED = 'rgba(136, 145, 168, 0.9)'
const GRID_COLOR = 'rgba(255, 255, 255, 0.06)'
const HISTOGRAM_BINS = 15

// Same rgb values as renderer/panels.ts's FILTER_COLORS (and the
// normalized equivalents in renderer/gl_utils.ts's FILTER_COLOR_RGB used
// for the 3D scene) — one filter runs per MC campaign (the Filter select
// above), so the NEES/NIS line should read as "that filter's color"
// consistently with every other place a filter's color shows up in the app.
const MC_FILTER_LINE_COLORS: Record<MCFilterKind, string> = {
  [MCFilterKind.Kf]: 'rgb(91, 140, 255)',
  [MCFilterKind.Ekf]: 'rgb(45, 217, 196)',
  [MCFilterKind.Ukf]: 'rgb(247, 169, 62)',
}
const MC_FILTER_LABELS: Record<MCFilterKind, string> = {
  [MCFilterKind.Kf]: 'KF',
  [MCFilterKind.Ekf]: 'EKF',
  [MCFilterKind.Ukf]: 'UKF',
}

const DEFAULTS = {
  nRuns: 500,
  filter: MCFilterKind.Ekf,
  nSteps: 500,
  dt: 10.0,
  qPos: 1.0,
  qVel: 0.01,
  seed: 42,
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

// RMS table rows sample the campaign's per-step series at a handful of
// evenly-spaced step indices rather than every step, distinct from the
// line-chart treatment used for NEES/NIS (those plot every step). Always
// includes step 0 and the last step, with up to `count - 2` more spread
// evenly between them — not fixed fractions of the run (0.25/0.5/0.75/1.0
// of nSteps), which is what this replaced: with a small nSteps (Steps is
// now a user-configurable field, not always the original fixed 500), those
// fractions round to indices that skip step 0 entirely (e.g. nSteps=5
// rounds 0.25*4=1, never 0), which read as "the table is missing the
// start of the run" — confirmed by a live report of exactly that.
//
// Three regimes verified by hand: nSteps=1 collapses every fraction to
// index 0 — old code produced 4 duplicate rows; new code dedupes via the
// Set, producing exactly 1. nSteps < count (e.g. 3) produces one row per
// step, all distinct, still always including 0 and nSteps-1. nSteps >=
// count (typical — default is 500) produces `count` evenly-spaced rows
// from 0 to nSteps-1 inclusive.
function sampleStepIndices(nSteps: number, count: number): number[] {
  if (nSteps <= 0) return []
  const n = Math.min(count, nSteps)
  const indices = new Set<number>()
  for (let i = 0; i < n; i++) {
    indices.add(n === 1 ? 0 : Math.round((i * (nSteps - 1)) / (n - 1)))
  }
  return [...indices].sort((a, b) => a - b)
}

function histogramBins(values: number[], bins: number): { labels: string[]; counts: number[] } {
  if (values.length === 0) return { labels: [], counts: [] }
  const min = Math.min(...values)
  const max = Math.max(...values)
  const width = (max - min) / bins || 1
  const counts = new Array<number>(bins).fill(0)
  for (const v of values) {
    const idx = Math.min(bins - 1, Math.floor((v - min) / width))
    counts[idx]!++
  }
  const labels = counts.map((_, i) => (min + i * width).toFixed(0))
  return { labels, counts }
}

// "500 steps x 10s" alone doesn't convey much without doing the
// multiplication — mirrors scenario_editor.ts's formatSimSpeedReadout
// precedent of translating a raw parameter into a plain-language readout.
function formatDurationReadout(nSteps: number, dt: number): string {
  if (!Number.isFinite(nSteps) || nSteps <= 0 || !Number.isFinite(dt) || dt <= 0) return ''
  const totalSec = nSteps * dt
  if (totalSec < 60) return `≈ ${totalSec.toFixed(0)} s simulated per run`
  if (totalSec < 3600) return `≈ ${(totalSec / 60).toFixed(1)} min simulated per run`
  return `≈ ${(totalSec / 3600).toFixed(2)} hr simulated per run`
}

export class MCResultsPanel {
  private readonly postToWorker: (msg: WorkerRequest) => void
  private readonly runsInput: HTMLInputElement
  private readonly filterSelect: HTMLSelectElement
  private readonly stepsInput: HTMLInputElement
  private readonly dtInput: HTMLInputElement
  private readonly durationReadout: HTMLSpanElement
  private readonly qPosInput: HTMLInputElement
  private readonly qVelInput: HTMLInputElement
  private readonly seedInput: HTMLInputElement
  private readonly randomizeCheckbox: HTMLInputElement
  private readonly statusLine: HTMLDivElement
  private readonly progressTrack: HTMLDivElement
  private readonly progressFill: HTMLDivElement
  private readonly runButton: HTMLButtonElement
  private readonly histogramChart: Chart
  private readonly neesChart: Chart
  private readonly nisChart: Chart
  private readonly rmsTableBody: HTMLTableSectionElement
  // Set when a campaign is dispatched, cleared once its 'mc_results'
  // response arrives — main.ts's render loop checks isRunning() every
  // frame to decide whether to keep polling the progress counter (see
  // McProgressReader) at all.
  private running = false
  private targetNRuns = 0
  // dt actually used by the in-flight/last campaign — MCStats doesn't echo
  // this back, so handleResults() needs it remembered from onRunMC() to
  // label the RMS table / NEES/NIS chart x-axes correctly (was hardcoded
  // to 10.0 back when dt itself was hardcoded).
  private lastDt = DEFAULTS.dt
  // Filter actually used by the in-flight/last campaign — same reason as
  // lastDt above (MCStats doesn't echo it back), needed by handleResults()
  // to color the NEES/NIS lines to match.
  private lastFilter: MCFilterKind = DEFAULTS.filter
  // Whether ScenarioEditor.getConfig() would currently succeed (a
  // satellite/TLE is loaded) — gates the button itself, same signal and
  // wording RunControls' own runEnabled uses. Independent of whether
  // init_scenario() has actually been called yet (see `initialized`
  // below): a config existing doesn't mean the engine has consumed it.
  private runEnabled = false
  // True once the live Simulation has been init_scenario()'d at least
  // once — either because the user clicked the topbar Run button, or
  // because onRunMC() did it itself on a first Run MC click (see
  // onRunMC()). Monte Carlo's initial condition is a snapshot of the live
  // Simulation's true state (x_true_initial_, wasm_api.hpp), only
  // populated by that call — before it's ever run once, a campaign would
  // run against a zeroed-out state vector and produce all-NaN results.
  private initialized = false
  private readonly getConfig: () => ScenarioConfig | undefined

  constructor(container: HTMLElement, options: MCResultsOptions) {
    this.postToWorker = options.postToWorker
    this.getConfig = options.getConfig

    const details = el('details', 'mc-panel')
    const summary = el('summary')
    summary.textContent = 'Monte Carlo'
    details.appendChild(summary)

    const body = el('div', 'mc-body')

    // Toolbar: each parameter is its own column (label on top, control
    // below). #mc-results-container is a full-width sibling of #panels
    // (the chart row above it), not nested in the narrow sidebar — so
    // these columns are sized to spread across that full width (CSS grid,
    // auto-fit) rather than packing tight on the left.
    const toolbar = el('div', 'mc-params-toolbar')

    this.runsInput = el('input')
    this.runsInput.type = 'number'
    this.runsInput.className = 'mc-number-input'
    this.runsInput.min = '1'
    this.runsInput.step = '1'
    this.runsInput.value = String(DEFAULTS.nRuns)
    toolbar.appendChild(this.makeField('Runs', this.runsInput,
      'Number of independent realizations in this campaign. More runs make the NEES/NIS averages and ' +
      'the histogram smoother, at a roughly linear cost in time.',
    ))

    this.filterSelect = el('select')
    for (const value of [MCFilterKind.Kf, MCFilterKind.Ekf, MCFilterKind.Ukf]) {
      const opt = el('option')
      opt.value = String(value)
      opt.textContent = MC_FILTER_LABELS[value]
      this.filterSelect.appendChild(opt)
    }
    this.filterSelect.value = String(DEFAULTS.filter)
    toolbar.appendChild(this.makeField('Filter', this.filterSelect,
      'Which filter this campaign runs, against the same noisy measurements each realization. KF is the ' +
      'intentionally-naive linearized baseline; EKF and UKF use the full nonlinear dynamics.',
    ))

    this.stepsInput = el('input')
    this.stepsInput.type = 'number'
    this.stepsInput.className = 'mc-number-input'
    this.stepsInput.min = '1'
    this.stepsInput.step = '1'
    this.stepsInput.value = String(DEFAULTS.nSteps)
    toolbar.appendChild(this.makeField('Steps', this.stepsInput,
      'Number of filter steps in one realization. Together with dt, sets how much simulated time one ' +
      'run covers (steps × dt).',
    ))

    this.dtInput = el('input')
    this.dtInput.type = 'number'
    this.dtInput.className = 'mc-number-input'
    this.dtInput.min = '0.1'
    this.dtInput.step = '0.1'
    this.dtInput.value = String(DEFAULTS.dt)
    toolbar.appendChild(this.makeField('Step size (s)', this.dtInput,
      'Simulated time between filter steps. Smaller values are finer-grained but need more steps to ' +
      'cover the same total duration.',
    ))

    this.qPosInput = el('input')
    this.qPosInput.type = 'number'
    this.qPosInput.className = 'mc-number-input'
    this.qPosInput.min = '0'
    this.qPosInput.step = '0.1'
    this.qPosInput.value = String(DEFAULTS.qPos)
    toolbar.appendChild(this.makeField('Process noise: position (m)', this.qPosInput,
      'Random position drift injected into the true trajectory every step, in addition to sensor noise ' +
      '— separate from the Scenario Editor. Higher values mean a harder target for the filter to track.',
    ))

    this.qVelInput = el('input')
    this.qVelInput.type = 'number'
    this.qVelInput.className = 'mc-number-input'
    this.qVelInput.min = '0'
    this.qVelInput.step = '0.001'
    this.qVelInput.value = String(DEFAULTS.qVel)
    toolbar.appendChild(this.makeField('Process noise: speed (m/s)', this.qVelInput,
      'Same as the position process noise, but for velocity drift.',
    ))

    this.seedInput = el('input')
    this.seedInput.type = 'number'
    this.seedInput.className = 'mc-number-input'
    this.seedInput.min = '0'
    this.seedInput.step = '1'
    this.seedInput.value = String(DEFAULTS.seed)
    const seedField = this.makeField('Random seed', this.seedInput,
      "Seed for this campaign's random noise draws. The same seed reproduces an identical campaign every " +
      'time; check Randomize below for a fresh draw each run.',
    )
    const randomizeRow = el('div', 'mc-randomize-row')
    const randomizeLabelEl = el('label')
    this.randomizeCheckbox = el('input')
    this.randomizeCheckbox.type = 'checkbox'
    randomizeLabelEl.append(this.randomizeCheckbox, document.createTextNode(' Randomize'))
    this.randomizeCheckbox.addEventListener('change', () => {
      this.seedInput.disabled = this.randomizeCheckbox.checked
    })
    randomizeRow.appendChild(randomizeLabelEl)
    seedField.appendChild(randomizeRow)
    toolbar.appendChild(seedField)

    const durationReadoutRow = el('div', 'mc-duration-readout-row')
    this.durationReadout = el('span', 'mc-duration-readout')
    this.durationReadout.textContent = formatDurationReadout(DEFAULTS.nSteps, DEFAULTS.dt)
    durationReadoutRow.appendChild(this.durationReadout)
    const refreshDurationReadout = (): void => {
      this.durationReadout.textContent = formatDurationReadout(
        Number(this.stepsInput.value), Number(this.dtInput.value),
      )
    }
    this.stepsInput.addEventListener('input', refreshDurationReadout)
    this.dtInput.addEventListener('input', refreshDurationReadout)

    const actionRow = el('div', 'row')
    this.runButton = el('button')
    this.runButton.textContent = '▶ Run MC'
    this.runButton.addEventListener('click', () => this.onRunMC())
    actionRow.appendChild(this.runButton)
    this.updateRunButtonState()

    this.statusLine = el('div', 'status-line')
    this.progressTrack = el('div', 'mc-progress-track')
    this.progressFill = el('div', 'mc-progress-fill')
    this.progressTrack.appendChild(this.progressFill)
    this.progressTrack.style.display = 'none'

    const chartsRow = el('div', 'mc-charts-row')

    const histogramCard = el('div', 'panel')
    const histogramTitle = el('h4', 'panel-title')
    histogramTitle.textContent = 'Final Position Error (histogram)'
    const histogramCanvas = el('canvas')
    histogramCard.append(histogramTitle, histogramCanvas)

    const rmsCard = el('div', 'panel')
    const rmsTitle = el('h4', 'panel-title')
    rmsTitle.textContent = 'RMS Error (table)'
    const rmsTable = el('table', 'mc-rms-table')
    const thead = el('thead')
    const headRow = el('tr')
    for (const h of ['t [s]', 'RMS pos [m]', 'RMS vel [m/s]']) {
      const th = el('th')
      th.textContent = h
      headRow.appendChild(th)
    }
    thead.appendChild(headRow)
    this.rmsTableBody = el('tbody')
    rmsTable.append(thead, this.rmsTableBody)
    rmsCard.append(rmsTitle, rmsTable)

    const neesCard = el('div', 'panel')
    const neesTitle = el('h4', 'panel-title')
    neesTitle.textContent = 'NEES (consistency)'
    const neesCanvas = el('canvas')
    neesCard.append(neesTitle, neesCanvas)

    const nisCard = el('div', 'panel')
    const nisTitle = el('h4', 'panel-title')
    nisTitle.textContent = 'NIS (consistency)'
    const nisCanvas = el('canvas')
    nisCard.append(nisTitle, nisCanvas)

    chartsRow.append(histogramCard, rmsCard, neesCard, nisCard)

    body.append(toolbar, durationReadoutRow, actionRow, this.statusLine, this.progressTrack, chartsRow)
    details.appendChild(body)
    container.appendChild(details)

    this.histogramChart = this.makeBarChart(histogramCanvas, 'Position error [m]', 'Count')
    this.neesChart = this.makeBoundedLineChart(neesCanvas, 'NEES')
    this.nisChart = this.makeBoundedLineChart(nisCanvas, 'NIS')
  }

  // One toolbar column: a small label (+ info button) on top, the given
  // control (input/select) below. control is built by the caller since
  // each one needs different type/min/step/options wiring.
  private makeField(labelText: string, control: HTMLElement, explanation: string): HTMLElement {
    const field = el('div', 'mc-field')
    const labelRow = el('div', 'mc-field-label')
    const label = el('label')
    label.textContent = labelText
    labelRow.append(label, makeInfoButton(explanation))
    field.append(labelRow, control)
    return field
  }

  private makeBarChart(canvas: HTMLCanvasElement, xTitle: string, yTitle: string): Chart {
    const ctx = canvas.getContext('2d')
    if (!ctx) throw new Error('canvas 2d context unavailable')
    const axisTitle = (text: string) => ({ display: true, text, color: TEXT_MUTED, font: { size: 9 } })
    return new Chart(ctx, {
      type: 'bar',
      data: { labels: [], datasets: [{ label: 'runs', data: [], backgroundColor: 'rgba(91,140,255,0.6)' }] },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: { type: 'category', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 }, maxTicksLimit: 5 }, title: axisTitle(xTitle) },
          y: { type: 'linear', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 } }, title: axisTitle(yTitle) },
        },
        plugins: {
          legend: { display: false },
          tooltip: { backgroundColor: '#111626', borderColor: '#232a40', borderWidth: 1 },
        },
      },
    })
  }

  private makeBoundedLineChart(canvas: HTMLCanvasElement, yTitle: string): Chart {
    const ctx = canvas.getContext('2d')
    if (!ctx) throw new Error('canvas 2d context unavailable')
    const axisTitle = (text: string) => ({ display: true, text, color: TEXT_MUTED, font: { size: 9 } })
    return new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'value', data: [], borderColor: 'rgb(45,217,196)', pointRadius: 0, borderWidth: 1.5 },
          { label: 'bounds', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
          { label: '_upper', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          // x is step index/time, formatted as "T+ (s)" the same way
          // panels.ts's main charts and this panel's own RMS table label
          // it — consistent across every place a campaign's time axis shows up.
          x: { type: 'category', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 }, maxTicksLimit: 5 }, title: axisTitle('T+ (s)') },
          y: { type: 'linear', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 } }, title: axisTitle(yTitle) },
        },
        plugins: {
          legend: {
            display: true,
            position: 'top',
            align: 'end',
            labels: { boxWidth: 8, boxHeight: 8, color: TEXT_MUTED, font: { size: 10 }, filter: (item) => !item.text.startsWith('_') },
          },
          tooltip: { backgroundColor: '#111626', borderColor: '#232a40', borderWidth: 1 },
        },
      },
    })
  }

  private onRunMC(): void {
    if (!this.runEnabled || this.running) return

    // First Run MC click ever (or the first since a fresh page load):
    // init the live Simulation ourselves rather than requiring the user
    // to have separately clicked the topbar Run button first — that
    // extra click had no visible connection to "fill out these fields
    // and click Run MC" from the user's side. Subsequent clicks skip
    // this; re-initializing on every click would reset live sim state
    // (T+, orbit trail) out from under a user who's mid-run and just
    // wants another MC campaign.
    if (!this.initialized) {
      const cfg = this.getConfig()
      if (!cfg) return // runEnabled implies this should never happen
      this.postToWorker({ type: 'init', payload: cfg })
      this.initialized = true
    }

    const nRuns = Math.max(1, Math.round(Number(this.runsInput.value) || DEFAULTS.nRuns))
    const nSteps = Math.max(1, Math.round(Number(this.stepsInput.value) || DEFAULTS.nSteps))
    const dt = Math.max(0.1, Number(this.dtInput.value) || DEFAULTS.dt)
    const qPos = Math.max(0, Number(this.qPosInput.value) || 0)
    const qVel = Math.max(0, Number(this.qVelInput.value) || 0)
    const filter = Number(this.filterSelect.value) as MCFilterKind
    const seed = this.randomizeCheckbox.checked ? -1 : Math.max(0, Math.round(Number(this.seedInput.value) || DEFAULTS.seed))

    this.targetNRuns = nRuns
    this.lastDt = dt
    this.lastFilter = filter
    this.running = true
    this.updateRunButtonState()
    this.progressTrack.style.display = ''
    this.updateProgress(0)
    this.postToWorker({ type: 'run_monte_carlo', payload: { nRuns, seed, filter, nSteps, dt, qPos, qVel } })
  }

  /** True between dispatching a campaign and its 'mc_results' response — main.ts's render loop polls progress only while this holds. */
  isRunning(): boolean {
    return this.running
  }

  /** Called by ScenarioEditor whenever getConfig() availability changes — same signal/wording RunControls' own setRunEnabled uses. */
  setRunEnabled(enabled: boolean): void {
    this.runEnabled = enabled
    this.updateRunButtonState()
  }

  /** Called by main.ts whenever an 'init' message goes to the worker (including ones triggered by the topbar Run button), so onRunMC() knows not to redundantly init again itself. */
  markInitialized(): void {
    this.initialized = true
  }

  private updateRunButtonState(): void {
    const disabled = !this.runEnabled || this.running
    this.runButton.disabled = disabled
    this.runButton.title = this.runEnabled ? '' : 'Select a satellite or paste a TLE first'
  }

  /** Called by main.ts every render frame while isRunning(), with the live mc_progress_counter() value. */
  updateProgress(completed: number): void {
    if (!this.running) return
    const clamped = Math.min(completed, this.targetNRuns)
    this.statusLine.textContent = `Running ${clamped} / ${this.targetNRuns} realizations...`
    const pct = this.targetNRuns > 0 ? (100 * clamped) / this.targetNRuns : 0
    this.progressFill.style.width = `${pct}%`
  }

  /** Called by main.ts when a 'mc_results' WorkerResponse arrives. */
  handleResults(stats: MCStats): void {
    this.running = false
    this.updateRunButtonState()
    this.progressTrack.style.display = 'none'
    const nRuns = stats.finalPosErrPerRun.length
    this.statusLine.textContent = `Done — ${nRuns} runs, ${stats.rmsPosPerStep.length} steps.`

    const { labels: histLabels, counts: histCounts } = histogramBins(stats.finalPosErrPerRun, HISTOGRAM_BINS)
    this.histogramChart.data.labels = histLabels
    this.histogramChart.data.datasets[0]!.data = histCounts
    this.histogramChart.update()

    this.rmsTableBody.replaceChildren()
    const nSteps = stats.rmsPosPerStep.length
    for (const idx of sampleStepIndices(nSteps, 4)) {
      const row = el('tr')
      for (const text of [
        ((idx + 1) * this.lastDt).toFixed(0),
        stats.rmsPosPerStep[idx]!.toFixed(2),
        stats.rmsVelPerStep[idx]!.toFixed(4),
      ]) {
        const td = el('td')
        td.textContent = text
        row.appendChild(td)
      }
      this.rmsTableBody.appendChild(row)
    }

    const stepLabels = stats.neesPerStep.map((_, i) => ((i + 1) * this.lastDt).toFixed(0))
    this.updateBoundedChart(this.neesChart, stepLabels, stats.neesPerStep, stats.neesLower, stats.neesUpper)
    this.updateBoundedChart(this.nisChart, stepLabels, stats.nisPerStep, stats.nisLower, stats.nisUpper)
  }

  private updateBoundedChart(chart: Chart, labels: string[], values: number[], lower: number, upper: number): void {
    chart.data.labels = labels
    chart.data.datasets[0]!.data = values
    chart.data.datasets[0]!.borderColor = MC_FILTER_LINE_COLORS[this.lastFilter]
    chart.data.datasets[0]!.label = MC_FILTER_LABELS[this.lastFilter]
    chart.data.datasets[1]!.data = labels.map(() => lower)
    chart.data.datasets[2]!.data = labels.map(() => upper)
    chart.update()
  }
}
