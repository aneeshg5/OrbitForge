// Chart.js streaming panels, KF/EKF/UKF simultaneously. Each of the 4
// panel slots has a <select> (wired in main.ts/index.html) letting the
// user pick which metric that slot shows — position/velocity error (norm
// or individual X/Y/Z component), covariance trace, NIS, and the Phase 5
// attitude metrics (error angle, angular velocity error norm/component).
// All metrics for all filters are buffered every frame regardless of
// which is currently selected, so switching a panel's dropdown is just a
// re-render, not a data-loss event.

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
import type { QuatCoeffs, StateFrame } from '../bridge/wasm_types.js'

Chart.register(LineController, LineElement, PointElement, LinearScale, CategoryScale, Legend, Tooltip)

const NIS_LOWER_BOUND = 0.216
const NIS_UPPER_BOUND = 9.348
const MAX_POINTS = 300

type FilterKey = 'kf' | 'ekf' | 'ukf'
const FILTER_KEYS: readonly FilterKey[] = ['kf', 'ekf', 'ukf']

function vecNorm3(a: readonly [number, number, number], b: readonly [number, number, number]): number {
  const dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2]
  return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

// KF's cov diag is [r(3),v(3)] (6 entries); EKF/UKF's is Phase 5's
// [delta_theta(3),omega(3),r(3),v(3)] (12 entries). The orbital [r,v]
// block is always the LAST 6 entries in both layouts, so summing diag's
// tail gives the same "position+velocity uncertainty" quantity for all
// three filters — summing the attitude block in too would add rad²/
// (rad/s)² to m²/(m/s)², which isn't a meaningful single number.
function orbitalCovTrace(diag: readonly number[]): number {
  return diag.slice(-6).reduce((sum, v) => sum + v, 0)
}

// Hamilton quaternion conjugate/product, coeffs() order (x,y,z,w) — same
// convention as math/quaternion.hpp. Used only for the attitude-error-angle
// metric below; not worth a shared module for two small pure functions.
function quatConj(q: QuatCoeffs): QuatCoeffs {
  return [-q[0], -q[1], -q[2], q[3]]
}
function quatMul(a: QuatCoeffs, b: QuatCoeffs): QuatCoeffs {
  const [ax, ay, az, aw] = a
  const [bx, by, bz, bw] = b
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ]
}

/** Angle (degrees) between two attitudes — 2*acos(|w| of the error quaternion). */
function quatAngleErrorDeg(qTrue: QuatCoeffs, qEst: QuatCoeffs): number {
  const err = quatMul(quatConj(qTrue), qEst)
  const w = Math.min(1, Math.max(-1, Math.abs(err[3])))
  return 2 * Math.acos(w) * (180 / Math.PI)
}

// A Metric knows how to pull one number out of a StateFrame for a given
// filter — `null` means "not applicable" (KF has no attitude state at all,
// §6.1, so every attitude metric is null for 'kf'; Chart.js renders a gap
// rather than a misleading zero).
interface Metric {
  key: string
  label: string
  unit: string
  compute: (frame: StateFrame, filter: FilterKey) => number | null
}

function posOf(frame: StateFrame, filter: FilterKey): readonly [number, number, number] {
  return filter === 'kf' ? frame.kfPos : filter === 'ekf' ? frame.ekfPos : frame.ukfPos
}
function velOf(frame: StateFrame, filter: FilterKey): readonly [number, number, number] {
  return filter === 'kf' ? frame.kfVel : filter === 'ekf' ? frame.ekfVel : frame.ukfVel
}
function omegaOf(frame: StateFrame, filter: FilterKey): readonly [number, number, number] | null {
  return filter === 'kf' ? null : filter === 'ekf' ? frame.ekfOmega : frame.ukfOmega
}
function quatOf(frame: StateFrame, filter: FilterKey): QuatCoeffs | null {
  return filter === 'kf' ? null : filter === 'ekf' ? frame.ekfQuat : frame.ukfQuat
}
function covDiagOf(frame: StateFrame, filter: FilterKey): readonly number[] {
  return filter === 'kf' ? frame.kfCovDiag : filter === 'ekf' ? frame.ekfCovDiag : frame.ukfCovDiag
}
function nisOf(frame: StateFrame, filter: FilterKey): number {
  return filter === 'kf' ? frame.kfNis : filter === 'ekf' ? frame.ekfNis : frame.ukfNis
}

const AXIS_LABEL = ['X', 'Y', 'Z'] as const

const METRICS: readonly Metric[] = [
  { key: 'posErrNorm', label: 'Position Error (norm)', unit: 'm', compute: (f, k) => vecNorm3(f.truePos, posOf(f, k)) },
  ...AXIS_LABEL.map((axis, i): Metric => ({
    key: `posErr${axis}`,
    label: `Position Error (${axis})`,
    unit: 'm',
    compute: (f, k) => posOf(f, k)[i]! - f.truePos[i]!,
  })),
  { key: 'velErrNorm', label: 'Velocity Error (norm)', unit: 'm/s', compute: (f, k) => vecNorm3(f.trueVel, velOf(f, k)) },
  ...AXIS_LABEL.map((axis, i): Metric => ({
    key: `velErr${axis}`,
    label: `Velocity Error (${axis})`,
    unit: 'm/s',
    compute: (f, k) => velOf(f, k)[i]! - f.trueVel[i]!,
  })),
  { key: 'covTrace', label: 'Covariance Trace', unit: '', compute: (f, k) => orbitalCovTrace(covDiagOf(f, k)) },
  { key: 'nis', label: 'NIS (consistency)', unit: '', compute: (f, k) => nisOf(f, k) },
  {
    key: 'attErrAngle',
    label: 'Attitude Error (angle)',
    unit: 'deg',
    compute: (f, k) => {
      const q = quatOf(f, k)
      return q ? quatAngleErrorDeg(f.trueQuat, q) : null
    },
  },
  {
    key: 'omegaErrNorm',
    label: 'Angular Velocity Error (norm)',
    unit: 'rad/s',
    compute: (f, k) => {
      const w = omegaOf(f, k)
      return w ? vecNorm3(f.trueOmega, w) : null
    },
  },
  ...AXIS_LABEL.map((axis, i): Metric => ({
    key: `omegaErr${axis}`,
    label: `Angular Velocity Error (${axis})`,
    unit: 'rad/s',
    compute: (f, k) => {
      const w = omegaOf(f, k)
      return w ? w[i]! - f.trueOmega[i]! : null
    },
  })),
]

const DEFAULT_METRIC_KEYS = ['posErrNorm', 'velErrNorm', 'covTrace', 'nis'] as const

function pushCapped(arr: number[], v: number): void {
  arr.push(v)
  if (arr.length > MAX_POINTS) arr.shift()
}

// Matches the --accent-blue/--accent-teal/--accent-orange tokens in
// index.html so the chart series and the 3D orbit-path/covariance colors
// for KF/EKF/UKF read as the same three things throughout the UI.
const FILTER_COLORS: Record<FilterKey, string> = {
  kf: 'rgb(91, 140, 255)',
  ekf: 'rgb(45, 217, 196)',
  ukf: 'rgb(247, 169, 62)',
}

const TEXT_MUTED = 'rgba(136, 145, 168, 0.9)'
const GRID_COLOR = 'rgba(255, 255, 255, 0.06)'

interface Panel {
  chart: Chart
  select: HTMLSelectElement
  metricKey: string
}

export class PanelManager {
  private readonly times: number[] = []
  // series[metricKey][filter] -> buffered values, one array per metric per filter
  private readonly series = new Map<string, Record<FilterKey, number[]>>()
  private readonly panels: Panel[]

  constructor(
    canvases: readonly [HTMLCanvasElement, HTMLCanvasElement, HTMLCanvasElement, HTMLCanvasElement],
    selects: readonly [HTMLSelectElement, HTMLSelectElement, HTMLSelectElement, HTMLSelectElement],
  ) {
    for (const metric of METRICS) {
      this.series.set(metric.key, { kf: [], ekf: [], ukf: [] })
    }

    this.panels = canvases.map((canvas, i): Panel => {
      const select = selects[i]!
      for (const metric of METRICS) {
        const opt = document.createElement('option')
        opt.value = metric.key
        opt.textContent = metric.unit ? `${metric.label} [${metric.unit}]` : metric.label
        select.appendChild(opt)
      }
      const defaultKey = DEFAULT_METRIC_KEYS[i]!
      select.value = defaultKey
      const isNis = defaultKey === 'nis'
      const panel: Panel = { chart: this.makeLineChart(canvas, isNis), select, metricKey: defaultKey }
      select.addEventListener('change', () => {
        panel.metricKey = select.value
        panel.chart.destroy()
        panel.chart = this.makeLineChart(canvas, select.value === 'nis')
        this.renderPanel(panel)
      })
      return panel
    })
  }

  // Dataset labels are just "KF"/"EKF"/"UKF" — the panel's own <select>
  // carries the quantity + unit, so the legend doesn't repeat it three
  // times per chart. withBounds adds the NIS chi-squared dashed bounds —
  // only meaningful when the panel's selected metric is NIS.
  private makeLineChart(canvas: HTMLCanvasElement, withBounds: boolean): Chart {
    const ctx = canvas.getContext('2d')
    if (!ctx) throw new Error('canvas 2d context unavailable')
    const extraDatasets: ChartDataset<'line'>[] = withBounds
      ? [
          { label: 'bounds', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
          { label: '_upper', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
        ]
      : []
    return new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'KF', data: [], borderColor: FILTER_COLORS.kf, pointRadius: 0, borderWidth: 1.75, spanGaps: false },
          { label: 'EKF', data: [], borderColor: FILTER_COLORS.ekf, pointRadius: 0, borderWidth: 1.75, spanGaps: false },
          { label: 'UKF', data: [], borderColor: FILTER_COLORS.ukf, pointRadius: 0, borderWidth: 1.75, spanGaps: false },
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
    for (const metric of METRICS) {
      const buf = this.series.get(metric.key)!
      for (const filter of FILTER_KEYS) {
        const v = metric.compute(frame, filter)
        // NaN (not null) so Chart.js's spanGaps:false renders a real gap —
        // Chart.js treats `null` in the data array as a gap too, but the
        // buffered arrays here are plain number[]; NaN-as-gap is handled
        // by Chart.js the same way as null for category-scale line charts.
        pushCapped(buf[filter], v ?? NaN)
      }
    }
  }

  /** Re-renders all four panels from the buffered series. Call once per UI frame, not per StateFrame. */
  render(): void {
    for (const panel of this.panels) this.renderPanel(panel)
  }

  /** Drops all buffered history (e.g. on Reset) — the next render() call shows empty charts. */
  clear(): void {
    this.times.length = 0
    for (const buf of this.series.values()) {
      buf.kf.length = 0
      buf.ekf.length = 0
      buf.ukf.length = 0
    }
  }

  private renderPanel(panel: Panel): void {
    const labels = this.times.map((t) => t.toFixed(0))
    const buf = this.series.get(panel.metricKey)!
    const datasets = [buf.kf, buf.ekf, buf.ukf]
    if (panel.metricKey === 'nis') {
      datasets.push(labels.map(() => NIS_LOWER_BOUND), labels.map(() => NIS_UPPER_BOUND))
    }
    panel.chart.data.labels = labels
    datasets.forEach((data, i) => {
      const ds = panel.chart.data.datasets[i]
      if (ds) ds.data = data
    })
    panel.chart.update()
  }
}
