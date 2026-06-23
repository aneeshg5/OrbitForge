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

const EARTH_RADIUS_KM = 6378.1

function orbitalCovTrace(diag: readonly number[]): number {
  return diag.slice(-6).reduce((sum, v) => sum + v, 0)
}

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

function quatAngleErrorDeg(qTrue: QuatCoeffs, qEst: QuatCoeffs): number {
  const err = quatMul(quatConj(qTrue), qEst)
  const w = Math.min(1, Math.max(-1, Math.abs(err[3])))
  return 2 * Math.acos(w) * (180 / Math.PI)
}

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

const FILTER_COLORS: Record<FilterKey, string> = {
  kf: 'rgb(91, 140, 255)',
  ekf: 'rgb(45, 217, 196)',
  ukf: 'rgb(247, 169, 62)',
}
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
  true: number[]
}

export class PanelManager {
  private readonly times: number[] = []
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
        pushCapped(buf[filter], v ?? NaN)
      }
      if (metric.trueValue) pushCapped(buf.true, metric.trueValue(frame))
    }
  }

  render(): void {
    for (const panel of this.panels) this.renderPanel(panel)
  }

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
