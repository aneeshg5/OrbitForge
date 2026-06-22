// Monte Carlo results panel: a "Runs" slider + "Run MC"
// button, plus three result widgets once a campaign completes — a
// final-position-error histogram, an RMS table, and NEES/NIS consistency
// charts with the campaign's own chi-squared bounds (not the fixed
// single-run bounds renderer/panels.ts uses). Posts a 'run_monte_carlo'
// WorkerRequest through the caller-supplied callback; never calls ccall
// directly.

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
import type { MCStats } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'

Chart.register(
  BarController, BarElement, LineController, LineElement, PointElement,
  LinearScale, CategoryScale, Legend, Tooltip,
)

export interface MCResultsOptions {
  postToWorker: (msg: WorkerRequest) => void
}

const TEXT_MUTED = 'rgba(136, 145, 168, 0.9)'
const GRID_COLOR = 'rgba(255, 255, 255, 0.06)'
const HISTOGRAM_BINS = 15

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

// RMS table rows sample the campaign's per-step series at fixed fractions
// of its duration rather than every step, distinct from the line-chart
// treatment used for NEES/NIS.
const TABLE_FRACTIONS = [0.25, 0.5, 0.75, 1.0]

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

export class MCResultsPanel {
  private readonly postToWorker: (msg: WorkerRequest) => void
  private readonly runsInput: HTMLInputElement
  private readonly runsLabel: HTMLSpanElement
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

  constructor(container: HTMLElement, options: MCResultsOptions) {
    this.postToWorker = options.postToWorker

    const details = el('details', 'mc-panel')
    const summary = el('summary')
    summary.textContent = 'Monte Carlo'
    details.appendChild(summary)

    const body = el('div', 'mc-body')

    const runsRow = el('div', 'row')
    const runsLabelText = el('label')
    runsLabelText.textContent = 'Runs: '
    this.runsInput = el('input')
    this.runsInput.type = 'range'
    this.runsInput.min = '100'
    this.runsInput.max = '5000'
    this.runsInput.step = '100'
    this.runsInput.value = '500'
    this.runsLabel = el('span')
    this.runsLabel.textContent = '500'
    this.runsInput.addEventListener('input', () => {
      this.runsLabel.textContent = this.runsInput.value
    })
    this.runButton = el('button')
    this.runButton.textContent = '▶ Run MC'
    this.runButton.addEventListener('click', () => this.onRunMC())
    runsRow.append(runsLabelText, this.runsInput, this.runsLabel, this.runButton)

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
    body.append(runsRow, this.statusLine, this.progressTrack, chartsRow)
    details.appendChild(body)
    container.appendChild(details)

    this.histogramChart = this.makeBarChart(histogramCanvas)
    this.neesChart = this.makeBoundedLineChart(neesCanvas)
    this.nisChart = this.makeBoundedLineChart(nisCanvas)
  }

  private makeBarChart(canvas: HTMLCanvasElement): Chart {
    const ctx = canvas.getContext('2d')
    if (!ctx) throw new Error('canvas 2d context unavailable')
    return new Chart(ctx, {
      type: 'bar',
      data: { labels: [], datasets: [{ label: 'runs', data: [], backgroundColor: 'rgba(91,140,255,0.6)' }] },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: { type: 'category', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 }, maxTicksLimit: 5 } },
          y: { type: 'linear', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 } } },
        },
        plugins: {
          legend: { display: false },
          tooltip: { backgroundColor: '#111626', borderColor: '#232a40', borderWidth: 1 },
        },
      },
    })
  }

  private makeBoundedLineChart(canvas: HTMLCanvasElement): Chart {
    const ctx = canvas.getContext('2d')
    if (!ctx) throw new Error('canvas 2d context unavailable')
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
          x: { type: 'category', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 }, maxTicksLimit: 5 } },
          y: { type: 'linear', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 9 } } },
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
    const nRuns = Number(this.runsInput.value)
    this.targetNRuns = nRuns
    this.running = true
    this.runButton.disabled = true
    this.progressTrack.style.display = ''
    this.updateProgress(0)
    this.postToWorker({ type: 'run_monte_carlo', payload: { nRuns, seed: -1 } })
  }

  /** True between dispatching a campaign and its 'mc_results' response — main.ts's render loop polls progress only while this holds. */
  isRunning(): boolean {
    return this.running
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
    this.runButton.disabled = false
    this.progressTrack.style.display = 'none'
    const nRuns = stats.finalPosErrPerRun.length
    this.statusLine.textContent = `Done — ${nRuns} runs, ${stats.rmsPosPerStep.length} steps.`

    const { labels: histLabels, counts: histCounts } = histogramBins(stats.finalPosErrPerRun, HISTOGRAM_BINS)
    this.histogramChart.data.labels = histLabels
    this.histogramChart.data.datasets[0]!.data = histCounts
    this.histogramChart.update()

    this.rmsTableBody.replaceChildren()
    const nSteps = stats.rmsPosPerStep.length
    for (const frac of TABLE_FRACTIONS) {
      const idx = Math.min(nSteps - 1, Math.max(0, Math.round(frac * (nSteps - 1))))
      const row = el('tr')
      const dt = 10.0  // matches Simulation::run_monte_carlo's fixed dt (wasm_api.cpp)
      for (const text of [
        ((idx + 1) * dt).toFixed(0),
        stats.rmsPosPerStep[idx]!.toFixed(2),
        stats.rmsVelPerStep[idx]!.toFixed(4),
      ]) {
        const td = el('td')
        td.textContent = text
        row.appendChild(td)
      }
      this.rmsTableBody.appendChild(row)
    }

    const stepLabels = stats.neesPerStep.map((_, i) => ((i + 1) * 10).toFixed(0))
    this.updateBoundedChart(this.neesChart, stepLabels, stats.neesPerStep, stats.neesLower, stats.neesUpper)
    this.updateBoundedChart(this.nisChart, stepLabels, stats.nisPerStep, stats.nisLower, stats.nisUpper)
  }

  private updateBoundedChart(chart: Chart, labels: string[], values: number[], lower: number, upper: number): void {
    chart.data.labels = labels
    chart.data.datasets[0]!.data = values
    chart.data.datasets[1]!.data = labels.map(() => lower)
    chart.data.datasets[2]!.data = labels.map(() => upper)
    chart.update()
  }
}
