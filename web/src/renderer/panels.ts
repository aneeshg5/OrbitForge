// Chart.js streaming panels, KF/EKF/UKF simultaneously. Each of the 4
// panel slots has a <select> (wired in main.ts/index.html) letting the
// user pick which metric that slot shows. Two families of metric:
// - Error/consistency metrics (position/velocity error, covariance trace,
//   NIS, attitude error, angular velocity error) — these compare each
//   filter against truth implicitly (truth is the zero line), so only
//   KF/EKF/UKF need plotting.
// - Absolute-value metrics (altitude, speed, position/velocity components)
//   — these answer "what's actually happening," not "how wrong is the
//   estimate," so a fourth "True" reference line is plotted alongside
//   each filter's own estimate of that same quantity (Metric.trueValue).
// All metrics for all filters are buffered every frame regardless of
// which is currently selected, so switching a panel's dropdown is just a
// re-render, not a data-loss event.

import {
  Chart,
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  Legend,
  Tooltip,
  type ChartDataset,
} from 'chart.js'
import type { QuatCoeffs, StateFrame } from '../bridge/wasm_types.js'

Chart.register(LineController, LineElement, PointElement, LinearScale, Legend, Tooltip)

const NIS_LOWER_BOUND = 0.216
const NIS_UPPER_BOUND = 9.348
const MAX_POINTS = 300

type FilterKey = 'kf' | 'ekf' | 'ukf'
const FILTER_KEYS: readonly FilterKey[] = ['kf', 'ekf', 'ukf']

function vecNorm3(a: readonly [number, number, number], b: readonly [number, number, number]): number {
  const dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2]
  return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

function vecMag3(v: readonly [number, number, number]): number {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

// Earth's mean radius, km — mirrors engine/include/constants.hpp's k_re
// (6.3781e6 m), for the same altitude the physics already works in.
const EARTH_RADIUS_KM = 6378.1

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
// rather than a misleading zero). `trueValue`, when present, marks this as
// an absolute-value metric and supplies the filter-independent ground-
// truth reference line plotted alongside KF/EKF/UKF.
interface Metric {
  key: string
  label: string
  unit: string
  compute: (frame: StateFrame, filter: FilterKey) => number | null
  trueValue?: (frame: StateFrame) => number
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
  // Absolute-value metrics — "what's actually happening," each with a
  // True reference line alongside the per-filter estimate.
  {
    key: 'altitude',
    label: 'Altitude',
    unit: 'km',
    compute: (f, k) => vecMag3(posOf(f, k)) / 1000 - EARTH_RADIUS_KM,
    trueValue: (f) => vecMag3(f.truePos) / 1000 - EARTH_RADIUS_KM,
  },
  {
    key: 'speed',
    label: 'Speed',
    unit: 'm/s',
    compute: (f, k) => vecMag3(velOf(f, k)),
    trueValue: (f) => vecMag3(f.trueVel),
  },
  ...AXIS_LABEL.map((axis, i): Metric => ({
    key: `pos${axis}`,
    label: `Position (${axis})`,
    unit: 'm',
    compute: (f, k) => posOf(f, k)[i]!,
    trueValue: (f) => f.truePos[i]!,
  })),
  ...AXIS_LABEL.map((axis, i): Metric => ({
    key: `vel${axis}`,
    label: `Velocity (${axis})`,
    unit: 'm/s',
    compute: (f, k) => velOf(f, k)[i]!,
    trueValue: (f) => f.trueVel[i]!,
  })),

  // Error/consistency metrics — truth is implicitly the zero line.
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

const DEFAULT_METRIC_KEYS = ['altitude', 'speed', 'posErrNorm', 'nis'] as const

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
// Dashed, dim white — matches orbit.ts's true-path convention ("true path:
// white (dashed)", CLAUDE.md §12) so "True" reads as the same ground-truth
// reference everywhere in the UI, not a fourth filter.
const TRUE_COLOR = 'rgba(255, 255, 255, 0.6)'

const TEXT_MUTED = 'rgba(136, 145, 168, 0.9)'
const GRID_COLOR = 'rgba(255, 255, 255, 0.06)'

interface Panel {
  chart: Chart
  select: HTMLSelectElement
  metricKey: string
}

interface SeriesBuffers {
  kf: number[]
  ekf: number[]
  ukf: number[]
  // Only populated for metrics with Metric.trueValue; empty/unused otherwise.
  true: number[]
}

export class PanelManager {
  private readonly times: number[] = []
  // series[metricKey] -> buffered values, one array per filter (+ true) per metric
  private readonly series = new Map<string, SeriesBuffers>()
  private readonly panels: Panel[]

  constructor(
    canvases: readonly [HTMLCanvasElement, HTMLCanvasElement, HTMLCanvasElement, HTMLCanvasElement],
    selects: readonly [HTMLSelectElement, HTMLSelectElement, HTMLSelectElement, HTMLSelectElement],
  ) {
    for (const metric of METRICS) {
      this.series.set(metric.key, { kf: [], ekf: [], ukf: [], true: [] })
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
      const panel: Panel = { chart: this.makeLineChart(canvas, defaultKey), select, metricKey: defaultKey }
      select.addEventListener('change', () => {
        panel.metricKey = select.value
        panel.chart.destroy()
        panel.chart = this.makeLineChart(canvas, select.value)
        this.renderPanel(panel)
      })
      return panel
    })
  }

  private extraDatasetKind(metricKey: string): 'bounds' | 'true' | 'none' {
    if (metricKey === 'nis') return 'bounds'
    return METRICS.find((m) => m.key === metricKey)?.trueValue ? 'true' : 'none'
  }

  // Dataset labels are just "KF"/"EKF"/"UKF" — the panel's own <select>
  // carries the quantity + unit in its option text, but that's not visible
  // once a dropdown is closed, so the Y axis also gets its own title
  // (the unit) — and the X axis "T+ (s)", matching the topbar clock's
  // own label, so a glance at either axis alone still says what it means.
  // `extra` adds either the NIS chi-squared dashed bounds or the
  // absolute-value metrics' "True" reference line — mutually exclusive,
  // since NIS is never an absolute-value metric.
  private makeLineChart(canvas: HTMLCanvasElement, metricKey: string): Chart {
    const metric = METRICS.find((m) => m.key === metricKey)
    const extra = this.extraDatasetKind(metricKey)
    const ctx = canvas.getContext('2d')
    if (!ctx) throw new Error('canvas 2d context unavailable')
    const extraDatasets: ChartDataset<'line'>[] =
      extra === 'bounds'
        ? [
            { label: 'bounds', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
            { label: '_upper', data: [], borderColor: 'rgba(150,150,150,0.5)', borderDash: [4, 4], pointRadius: 0, borderWidth: 1 },
          ]
        : extra === 'true'
          ? [{ label: 'True', data: [], borderColor: TRUE_COLOR, borderDash: [3, 3], pointRadius: 0, borderWidth: 1.5, spanGaps: false }]
          : []
    return new Chart(ctx, {
      type: 'line',
      data: {
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
          // Linear (numeric, x/y point pairs), not category-by-string-label
          // — a category axis only ever shows ticks for labels that exist,
          // so before any data has streamed in (or in the first instant
          // after Run) the axis was completely blank. min/max are set
          // explicitly on every renderPanel() call below (not fixed here,
          // and deliberately not Chart.js's suggestedMin/Max) — suggested*
          // bounds widen the range to include them but never let go once
          // real data moves past them, which left the axis permanently
          // pinned at 0 while MAX_POINTS' sliding window of actual data
          // crept further and further right, squeezing 300 real points
          // into a sliver and leaving the rest of the chart blank.
          x: {
            type: 'linear',
            min: 0,
            max: 60,
            grid: { color: GRID_COLOR },
            ticks: { color: TEXT_MUTED, font: { size: 10 }, maxTicksLimit: 6 },
            title: { display: true, text: 'T+ (s)', color: TEXT_MUTED, font: { size: 10 } },
          },
          y: {
            type: 'linear',
            grid: { color: GRID_COLOR },
            ticks: { color: TEXT_MUTED, font: { size: 10 } },
            title: { display: !!metric?.unit, text: metric?.unit ?? '', color: TEXT_MUTED, font: { size: 10 } },
          },
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
        // Chart.js treats `null` y-values as a gap too, but the buffered
        // arrays here are plain number[]; NaN-as-gap is handled the same
        // way as null/missing for line charts regardless of x-axis type.
        pushCapped(buf[filter], v ?? NaN)
      }
      if (metric.trueValue) pushCapped(buf.true, metric.trueValue(frame))
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
      buf.true.length = 0
    }
  }

  private renderPanel(panel: Panel): void {
    // Pins the axis to exactly the currently-buffered window — the bug
    // this replaced (a fixed suggestedMin of 0) left the axis anchored at
    // 0 forever while MAX_POINTS' sliding window of real data drifted
    // further right as the run went on, squeezing 300 real points into a
    // sliver at the edge with the rest of the chart blank.
    const xScale = panel.chart.options.scales!.x!
    if (this.times.length === 0) {
      xScale.min = 0
      xScale.max = 60
    } else {
      xScale.min = this.times[0]
      xScale.max = this.times[this.times.length - 1]
    }

    const buf = this.series.get(panel.metricKey)!
    const zip = (values: readonly number[]): { x: number; y: number }[] =>
      values.map((y, idx) => ({ x: this.times[idx]!, y }))

    const datasets = [zip(buf.kf), zip(buf.ekf), zip(buf.ukf)]
    if (panel.metricKey === 'nis') {
      datasets.push(
        this.times.map((x) => ({ x, y: NIS_LOWER_BOUND })),
        this.times.map((x) => ({ x, y: NIS_UPPER_BOUND })),
      )
    } else if (buf.true.length > 0) {
      datasets.push(zip(buf.true))
    }
    datasets.forEach((data, i) => {
      const ds = panel.chart.data.datasets[i]
      if (ds) ds.data = data
    })
    panel.chart.update()
  }
}
