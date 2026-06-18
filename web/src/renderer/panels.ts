// Chart.js streaming panels: position error norm, velocity
// error norm, covariance trace, and NIS with chi-squared consistency
// bounds — all per StateFrame, for KF/EKF/UKF simultaneously.
//
// NIS bounds here are for a single live run (N=1), so the relevant
// reference is the per-step chi2(3) distribution directly (measurement
// dimension = 3 for GPS position), not the N-averaged bounds mc_runner.cpp
// computes for a Monte Carlo campaign — those only apply
// once N independent runs are averaged together. Standard chi-squared
// table values for 3 degrees of freedom: chi2(3, 0.025) = 0.216,
// chi2(3, 0.975) = 9.348.

import {
  Chart,
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Legend,
  Tooltip,
  type ChartDataset,
} from 'chart.js'
import type { StateFrame } from '../bridge/wasm_types.js'

Chart.register(LineController, LineElement, PointElement, LinearScale, CategoryScale, Legend, Tooltip)

const NIS_LOWER_BOUND = 0.216
const NIS_UPPER_BOUND = 9.348
const MAX_POINTS = 300

function norm3(a: readonly [number, number, number], b: readonly [number, number, number]): number {
  const dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2]
  return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

function covTrace(diag: readonly [number, number, number, number, number, number]): number {
  return diag[0] + diag[1] + diag[2] + diag[3] + diag[4] + diag[5]
}

interface FilterSeries {
  posErr: number[]
  velErr: number[]
  covTrace: number[]
  nis: number[]
}

function emptySeries(): FilterSeries {
  return { posErr: [], velErr: [], covTrace: [], nis: [] }
}

function pushCapped(arr: number[], v: number): void {
  arr.push(v)
  if (arr.length > MAX_POINTS) arr.shift()
}

// Matches the --accent-blue/--accent-teal/--accent-orange tokens in
// index.html so the chart series and the 3D orbit-path/covariance colors
// for KF/EKF/UKF read as the same three things throughout the UI.
const FILTER_COLORS = {
  kf: 'rgb(91, 140, 255)',
  ekf: 'rgb(45, 217, 196)',
  ukf: 'rgb(247, 169, 62)',
} as const

const TEXT_MUTED = 'rgba(136, 145, 168, 0.9)'
const GRID_COLOR = 'rgba(255, 255, 255, 0.06)'

export class PanelManager {
  private readonly times: number[] = []
  private readonly series: { kf: FilterSeries; ekf: FilterSeries; ukf: FilterSeries } = {
    kf: emptySeries(),
    ekf: emptySeries(),
    ukf: emptySeries(),
  }

  private readonly posErrChart: Chart
  private readonly velErrChart: Chart
  private readonly covTraceChart: Chart
  private readonly nisChart: Chart

  constructor(
    posErrCanvas: HTMLCanvasElement,
    velErrCanvas: HTMLCanvasElement,
    covTraceCanvas: HTMLCanvasElement,
    nisCanvas: HTMLCanvasElement,
  ) {
    this.posErrChart = this.makeLineChart(posErrCanvas)
    this.velErrChart = this.makeLineChart(velErrCanvas)
    this.covTraceChart = this.makeLineChart(covTraceCanvas)
    this.nisChart = this.makeLineChart(nisCanvas, [
      { label: 'bounds', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
      { label: '_upper', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
    ])
  }

  // Dataset labels are just "KF"/"EKF"/"UKF" — the panel's own <h4> title
  // (index.html) carries the quantity + unit, so the legend doesn't repeat
  // it three times per chart.
  private makeLineChart(canvas: HTMLCanvasElement, extraDatasets: ChartDataset<'line'>[] = []): Chart {
    const ctx = canvas.getContext('2d')
    if (!ctx) throw new Error('canvas 2d context unavailable')
    return new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'KF', data: [], borderColor: FILTER_COLORS.kf, pointRadius: 0, borderWidth: 1.75 },
          { label: 'EKF', data: [], borderColor: FILTER_COLORS.ekf, pointRadius: 0, borderWidth: 1.75 },
          { label: 'UKF', data: [], borderColor: FILTER_COLORS.ukf, pointRadius: 0, borderWidth: 1.75 },
          ...extraDatasets,
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        interaction: { mode: 'index', intersect: false },
        scales: {
          x: { type: 'category', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 10 }, maxTicksLimit: 6 } },
          y: { type: 'linear', grid: { color: GRID_COLOR }, ticks: { color: TEXT_MUTED, font: { size: 10 } } },
        },
        plugins: {
          legend: {
            display: true,
            position: 'top',
            align: 'end',
            labels: {
              boxWidth: 8,
              boxHeight: 8,
              color: TEXT_MUTED,
              font: { size: 11, family: "'Inter', system-ui, sans-serif" },
              filter: (item) => !item.text.startsWith('_'),
            },
          },
          tooltip: {
            backgroundColor: '#111626',
            borderColor: '#232a40',
            borderWidth: 1,
            titleFont: { family: "'Fira Code', monospace", size: 11 },
            bodyFont: { family: "'Fira Code', monospace", size: 11 },
          },
        },
      },
    })
  }

  pushFrame(frame: StateFrame): void {
    pushCapped(this.times, frame.simTime)

    pushCapped(this.series.kf.posErr, norm3(frame.truePos, frame.kfPos))
    pushCapped(this.series.kf.velErr, norm3(frame.trueVel, frame.kfVel))
    pushCapped(this.series.kf.covTrace, covTrace(frame.kfCovDiag))
    pushCapped(this.series.kf.nis, frame.kfNis)

    pushCapped(this.series.ekf.posErr, norm3(frame.truePos, frame.ekfPos))
    pushCapped(this.series.ekf.velErr, norm3(frame.trueVel, frame.ekfVel))
    pushCapped(this.series.ekf.covTrace, covTrace(frame.ekfCovDiag))
    pushCapped(this.series.ekf.nis, frame.ekfNis)

    pushCapped(this.series.ukf.posErr, norm3(frame.truePos, frame.ukfPos))
    pushCapped(this.series.ukf.velErr, norm3(frame.trueVel, frame.ukfVel))
    pushCapped(this.series.ukf.covTrace, covTrace(frame.ukfCovDiag))
    pushCapped(this.series.ukf.nis, frame.ukfNis)
  }

  /** Re-renders all four charts from the buffered series. Call once per UI frame, not per StateFrame. */
  render(): void {
    const labels = this.times.map((t) => t.toFixed(0))

    this.updateChart(this.posErrChart, labels, [this.series.kf.posErr, this.series.ekf.posErr, this.series.ukf.posErr])
    this.updateChart(this.velErrChart, labels, [this.series.kf.velErr, this.series.ekf.velErr, this.series.ukf.velErr])
    this.updateChart(this.covTraceChart, labels, [this.series.kf.covTrace, this.series.ekf.covTrace, this.series.ukf.covTrace])

    const lowerBound = labels.map(() => NIS_LOWER_BOUND)
    const upperBound = labels.map(() => NIS_UPPER_BOUND)
    this.updateChart(this.nisChart, labels, [this.series.kf.nis, this.series.ekf.nis, this.series.ukf.nis, lowerBound, upperBound])
  }

  private updateChart(chart: Chart, labels: string[], datasets: number[][]): void {
    chart.data.labels = labels
    datasets.forEach((data, i) => {
      const ds = chart.data.datasets[i]
      if (ds) ds.data = data
    })
    chart.update()
  }
}
