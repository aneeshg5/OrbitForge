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
  getConfig: () => ScenarioConfig | undefined
}

const TEXT_MUTED = 'rgba(136, 145, 168, 0.9)'
const GRID_COLOR = 'rgba(255, 255, 255, 0.06)'
const HISTOGRAM_BINS = 15

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
  private running = false
  private targetNRuns = 0
  private lastDt = DEFAULTS.dt
  private lastFilter: MCFilterKind = DEFAULTS.filter
  private runEnabled = false
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

    if (!this.initialized) {
      const cfg = this.getConfig()
      if (!cfg) return
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

  isRunning(): boolean {
    return this.running
  }

  setRunEnabled(enabled: boolean): void {
    this.runEnabled = enabled
    this.updateRunButtonState()
  }

  markInitialized(): void {
    this.initialized = true
  }

  private updateRunButtonState(): void {
    const disabled = !this.runEnabled || this.running
    this.runButton.disabled = disabled
    this.runButton.title = this.runEnabled ? '' : 'Select a satellite or paste a TLE first'
  }

  updateProgress(completed: number): void {
    if (!this.running) return
    const clamped = Math.min(completed, this.targetNRuns)
    this.statusLine.textContent = `Running ${clamped} / ${this.targetNRuns} realizations...`
    const pct = this.targetNRuns > 0 ? (100 * clamped) / this.targetNRuns : 0
    this.progressFill.style.width = `${pct}%`
  }

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
