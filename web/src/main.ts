import { RingReader } from './bridge/ring_reader.js'
import { McProgressReader } from './bridge/mc_progress_reader.js'
import type { StateFrame } from './bridge/wasm_types.js'
import type { WorkerRequest, WorkerResponse } from './worker.js'
import { EarthRenderer } from './renderer/earth.js'
import { RotationAxisRenderer } from './renderer/axis.js'
import { OrbitPathRenderer } from './renderer/orbit.js'
import { CovarianceEllipsoidRenderer } from './renderer/covariance.js'
import { SatelliteMarkerRenderer } from './renderer/satellite_marker.js'
import { AttitudeGizmoRenderer, AttitudeSmoother, type GizmoViewport } from './renderer/attitude.js'
import { Starfield } from './renderer/starfield.js'
import { SolarSystemRenderer, sunDirectionScene } from './renderer/solar_system.js'
import { PanelManager } from './renderer/panels.js'
import { mat4LookAt, mat4Multiply, mat4Perspective, mat4RotateY, mat4RotateZ, mat4StripTranslation, type Mat4, FILTER_COLOR_RGB } from './renderer/gl_utils.js'
import { ScenarioEditor } from './ui/scenario_editor.js'
import { FaultPanel } from './ui/fault_panel.js'
import { MCResultsPanel } from './ui/mc_results.js'
import { RunControls } from './ui/run_controls.js'

const EARTH_TILT = mat4RotateZ((23.4 * Math.PI) / 180)

const SPIN_RAD_PER_SIM_SECOND = (2 * Math.PI) / 86400

interface Scene {
  gl: WebGL2RenderingContext
  starfield: Starfield
  solarSystem: SolarSystemRenderer
  earth: EarthRenderer
  axis: RotationAxisRenderer
  orbits: OrbitPathRenderer
  covariances: CovarianceEllipsoidRenderer
  satelliteMarker: SatelliteMarkerRenderer
  attitude: AttitudeGizmoRenderer
  attitudeSmoother: AttitudeSmoother
  attitudeGizmoFrame: HTMLElement
  axisLabels: { x: HTMLElement; y: HTMLElement; z: HTMLElement }
  simTimeValue: HTMLElement
  panels: PanelManager
  canvas: HTMLCanvasElement
}

class OrbitCamera {
  private azimuth = 0.6
  private elevation = 0.4
  private distance = 4.0

  private velAzimuth = 0
  private velElevation = 0

  private static readonly k_drag_sensitivity = 0.005
  private static readonly k_damping = 0.92
  private static readonly k_min_distance = 1.5
  private static readonly k_max_distance = 40

  private dragging = false

  constructor(canvas: HTMLCanvasElement) {
    let lastX = 0
    let lastY = 0

    canvas.style.cursor = 'grab'
    canvas.style.touchAction = 'none'

    canvas.addEventListener('pointerdown', (e) => {
      this.dragging = true
      lastX = e.clientX
      lastY = e.clientY
      this.velAzimuth = 0
      this.velElevation = 0
      canvas.setPointerCapture(e.pointerId)
      canvas.style.cursor = 'grabbing'
      e.preventDefault()
    })

    canvas.addEventListener('pointermove', (e) => {
      if (!this.dragging) return
      const dx = e.clientX - lastX
      const dy = e.clientY - lastY
      lastX = e.clientX
      lastY = e.clientY

      const dAzimuth = -dx * OrbitCamera.k_drag_sensitivity
      const dElevation = -dy * OrbitCamera.k_drag_sensitivity
      this.azimuth += dAzimuth
      this.elevation = Math.max(-1.5, Math.min(1.5, this.elevation + dElevation))
      this.velAzimuth = dAzimuth
      this.velElevation = dElevation
      e.preventDefault()
    })

    const endDrag = (e: PointerEvent): void => {
      if (!this.dragging) return
      this.dragging = false
      canvas.style.cursor = 'grab'
      canvas.releasePointerCapture(e.pointerId)
    }
    canvas.addEventListener('pointerup', endDrag)
    canvas.addEventListener('pointercancel', endDrag)

    canvas.addEventListener('wheel', (e) => {
      this.distance = Math.max(
        OrbitCamera.k_min_distance,
        Math.min(OrbitCamera.k_max_distance, this.distance * (1 + e.deltaY * 0.001)),
      )
      e.preventDefault()
    }, { passive: false })
  }

  update(): void {
    if (this.dragging) return
    if (Math.abs(this.velAzimuth) < 1e-5 && Math.abs(this.velElevation) < 1e-5) return
    this.azimuth += this.velAzimuth
    this.elevation = Math.max(-1.5, Math.min(1.5, this.elevation + this.velElevation))
    this.velAzimuth *= OrbitCamera.k_damping
    this.velElevation *= OrbitCamera.k_damping
  }

  viewMatrix(): Mat4 {
    const eye: [number, number, number] = [
      this.distance * Math.cos(this.elevation) * Math.sin(this.azimuth),
      this.distance * Math.sin(this.elevation),
      this.distance * Math.cos(this.elevation) * Math.cos(this.azimuth),
    ]
    return mat4LookAt(eye, [0, 0, 0], [0, 1, 0])
  }
}

function setupScene(): Scene {
  const canvas = document.getElementById('scene-canvas') as HTMLCanvasElement
  const gl = canvas.getContext('webgl2')
  if (!gl) throw new Error('WebGL2 not available')

  const panelCanvases: [HTMLCanvasElement, HTMLCanvasElement, HTMLCanvasElement, HTMLCanvasElement] = [
    document.getElementById('pos-err-canvas') as HTMLCanvasElement,
    document.getElementById('vel-err-canvas') as HTMLCanvasElement,
    document.getElementById('cov-trace-canvas') as HTMLCanvasElement,
    document.getElementById('nis-canvas') as HTMLCanvasElement,
  ]
  const panelSelects: [HTMLSelectElement, HTMLSelectElement, HTMLSelectElement, HTMLSelectElement] = [
    document.getElementById('panel-select-0') as HTMLSelectElement,
    document.getElementById('panel-select-1') as HTMLSelectElement,
    document.getElementById('panel-select-2') as HTMLSelectElement,
    document.getElementById('panel-select-3') as HTMLSelectElement,
  ]

  return {
    gl,
    canvas,
    starfield: new Starfield(gl),
    solarSystem: new SolarSystemRenderer(gl),
    earth: new EarthRenderer(gl),
    axis: new RotationAxisRenderer(gl),
    orbits: new OrbitPathRenderer(gl),
    covariances: new CovarianceEllipsoidRenderer(gl),
    satelliteMarker: new SatelliteMarkerRenderer(gl),
    attitude: new AttitudeGizmoRenderer(gl),
    attitudeSmoother: new AttitudeSmoother(),
    attitudeGizmoFrame: document.getElementById('attitude-gizmo-frame')!,
    axisLabels: {
      x: document.getElementById('axis-label-x')!,
      y: document.getElementById('axis-label-y')!,
      z: document.getElementById('axis-label-z')!,
    },
    simTimeValue: document.getElementById('sim-time-value')!,
    panels: new PanelManager(panelCanvases, panelSelects),
  }
}

function computeGizmoViewport(canvas: HTMLCanvasElement, frame: HTMLElement): GizmoViewport {
  const canvasRect = canvas.getBoundingClientRect()
  const frameRect = frame.getBoundingClientRect()
  const scaleX = canvas.width / Math.max(1, canvasRect.width)
  const scaleY = canvas.height / Math.max(1, canvasRect.height)

  const width = frameRect.width * scaleX
  const height = frameRect.height * scaleY
  const x = (frameRect.left - canvasRect.left) * scaleX
  const topCss = (frameRect.top - canvasRect.top) * scaleY
  const y = canvas.height - topCss - height

  return { x, y, width, height }
}

function formatSimTime(totalSeconds: number): string {
  const s = Math.max(0, Math.floor(totalSeconds))
  const days = Math.floor(s / 86400)
  const hours = Math.floor((s % 86400) / 3600)
  const minutes = Math.floor((s % 3600) / 60)
  const seconds = s % 60
  const pad = (n: number): string => String(n).padStart(2, '0')
  const hms = `${pad(hours)}:${pad(minutes)}:${pad(seconds)}`
  return days > 0 ? `${days}:${hms}` : hms
}

function resizeCanvasToDisplaySize(canvas: HTMLCanvasElement): boolean {
  const dpr = window.devicePixelRatio || 1
  const width = Math.round(canvas.clientWidth * dpr)
  const height = Math.round(canvas.clientHeight * dpr)
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width
    canvas.height = height
    return true
  }
  return false
}

function feedFrameToScene(scene: Scene, frame: StateFrame): void {
  scene.orbits.addPoint('true', frame.truePos)
  scene.orbits.addPoint('kf', frame.kfPos)
  scene.orbits.addPoint('ekf', frame.ekfPos)
  scene.orbits.addPoint('ukf', frame.ukfPos)
  scene.panels.pushFrame(frame)
}

function renderScene(scene: Scene, camera: OrbitCamera, latestFrame: StateFrame | undefined): void {
  const { gl, canvas } = scene
  resizeCanvasToDisplaySize(canvas)
  gl.viewport(0, 0, canvas.width, canvas.height)
  gl.clearColor(0.02, 0.03, 0.06, 1.0)
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

  const aspect = canvas.width / Math.max(1, canvas.height)
  const proj = mat4Perspective((45 * Math.PI) / 180, aspect, 0.05, 25000)
  const view = mat4Multiply(camera.viewMatrix(), EARTH_TILT)

  const tSec = performance.now() / 1000

  const simTimeSec = latestFrame ? latestFrame.simTime : 0
  scene.simTimeValue.textContent = formatSimTime(simTimeSec)

  const spinAngle = (simTimeSec * SPIN_RAD_PER_SIM_SECOND) % (2 * Math.PI)
  const earthModel = mat4RotateY(spinAngle)

  const sunDirScene = sunDirectionScene(simTimeSec)

  const attitudeTargetQuat: StateFrame['trueQuat'] = latestFrame ? latestFrame.trueQuat : [0, 0, 0, 1]
  const smoothedQuat = scene.attitudeSmoother.update(attitudeTargetQuat)

  const viewRotation = mat4StripTranslation(view)
  scene.starfield.render(viewRotation, proj)

  scene.earth.render(earthModel, view, proj, sunDirScene)
  scene.axis.render(view, proj)
  scene.orbits.render(view, proj)

  if (latestFrame) {
    scene.covariances.render(view, proj, latestFrame.kfPos, [latestFrame.kfCovDiag[0], latestFrame.kfCovDiag[1], latestFrame.kfCovDiag[2]], FILTER_COLOR_RGB.kf)
    scene.covariances.render(view, proj, latestFrame.ekfPos, [latestFrame.ekfCovDiag[6], latestFrame.ekfCovDiag[7], latestFrame.ekfCovDiag[8]], FILTER_COLOR_RGB.ekf)
    scene.covariances.render(view, proj, latestFrame.ukfPos, [latestFrame.ukfCovDiag[6], latestFrame.ukfCovDiag[7], latestFrame.ukfCovDiag[8]], FILTER_COLOR_RGB.ukf)

    scene.satelliteMarker.render(view, proj, latestFrame.truePos, smoothedQuat, sunDirScene)
  }

  scene.solarSystem.render(view, proj, sunDirScene, tSec, simTimeSec)

  const gizmoViewport = computeGizmoViewport(scene.canvas, scene.attitudeGizmoFrame)
  const gizmoQuat = smoothedQuat
  scene.attitude.render(gizmoQuat, gizmoViewport, canvas.width, canvas.height)

  const gizmoAspect = gizmoViewport.width / Math.max(1, gizmoViewport.height)
  const axisTips = scene.attitude.computeAxisTipsNdc(gizmoQuat, gizmoAspect)
  const frameRect = scene.attitudeGizmoFrame.getBoundingClientRect()
  const containerRect = canvas.parentElement!.getBoundingClientRect()
  positionAxisLabel(scene.axisLabels.x, axisTips.x, frameRect, containerRect)
  positionAxisLabel(scene.axisLabels.y, axisTips.y, frameRect, containerRect)
  positionAxisLabel(scene.axisLabels.z, axisTips.z, frameRect, containerRect)

  scene.panels.render()
}

function positionAxisLabel(
  label: HTMLElement,
  ndc: readonly [number, number],
  frameRect: DOMRect,
  containerRect: DOMRect,
): void {
  const xWithinFrame = (ndc[0] * 0.5 + 0.5) * frameRect.width
  const yWithinFrame = (1 - (ndc[1] * 0.5 + 0.5)) * frameRect.height
  label.style.left = `${frameRect.left - containerRect.left + xWithinFrame}px`
  label.style.top = `${frameRect.top - containerRect.top + yWithinFrame}px`
}

function startRenderLoop(
  scene: Scene,
  camera: OrbitCamera,
  getRingReader: () => RingReader | undefined,
  getRunControls: () => RunControls | undefined,
  getMcProgress: () => McProgressReader | undefined,
  getMcResultsPanel: () => MCResultsPanel | undefined,
): { resetView: () => void; getCurrentSimTimeSec: () => number } {
  let latestFrame: StateFrame | undefined

  function clearView(): void {
    latestFrame = undefined
    scene.orbits.clear()
    scene.panels.clear()
  }

  function tick(): void {
    const ringReader = getRingReader()
    if (ringReader) {
      const { frames, reset } = ringReader.drain()
      if (reset) clearView()
      for (const frame of frames) {
        feedFrameToScene(scene, frame)
        latestFrame = frame
      }
    }
    camera.update()
    renderScene(scene, camera, latestFrame)
    getRunControls()?.checkAutoStop(latestFrame ? latestFrame.simTime : 0)

    const mcResultsPanel = getMcResultsPanel()
    if (mcResultsPanel?.isRunning()) {
      const progress = getMcProgress()
      if (progress) mcResultsPanel.updateProgress(progress.read())
    }

    requestAnimationFrame(tick)
  }
  requestAnimationFrame(tick)

  return { resetView: clearView, getCurrentSimTimeSec: () => (latestFrame ? latestFrame.simTime : 0) }
}

function registerServiceWorker(): void {
  if (import.meta.env.DEV) return
  if (!('serviceWorker' in navigator)) return
  navigator.serviceWorker.register('/sw.js').catch((err: unknown) => {
    console.warn('Service worker registration failed:', err)
  })
}

async function main(): Promise<void> {
  registerServiceWorker()

  const scene = setupScene()
  const camera = new OrbitCamera(scene.canvas)

  let runControls: RunControls | undefined
  let scenarioEditor: ScenarioEditor | undefined
  let ringReader: RingReader | undefined
  let mcProgress: McProgressReader | undefined
  let mcResults: MCResultsPanel | undefined
  const { resetView, getCurrentSimTimeSec } = startRenderLoop(
    scene, camera, () => ringReader, () => runControls, () => mcProgress, () => mcResults,
  )

  const worker = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' })
  const postToWorker = (msg: WorkerRequest): void => {
    if (msg.type === 'reset') resetView()
    if (msg.type === 'init') mcResults!.markInitialized()
    worker.postMessage(msg)
  }

  const mcContainer = document.getElementById('mc-results-container')!
  mcResults = new MCResultsPanel(mcContainer, { postToWorker, getConfig: () => scenarioEditor!.getConfig() })

  worker.addEventListener('message', (e: MessageEvent<WorkerResponse>) => {
    if (e.data.type === 'ring_buffer_ready') {
      const { sharedArrayBuffer, ringBufferPtr, ringBufferCapacity, mcProgressPtr } = e.data.payload
      ringReader = new RingReader(sharedArrayBuffer, ringBufferPtr, ringBufferCapacity)
      mcProgress = new McProgressReader(sharedArrayBuffer, mcProgressPtr)
    } else if (e.data.type === 'mc_results') {
      mcResults!.handleResults(e.data.payload)
    }
  })

  const scenarioContainer = document.getElementById('scenario-editor-container')!
  const faultContainer = document.getElementById('fault-panel-container')!
  new FaultPanel(faultContainer, { postToWorker, getCurrentSimTimeSec })

  const runControlsContainer = document.getElementById('run-controls-container')!
  runControls = new RunControls(runControlsContainer, {
    postToWorker,
    getConfig: () => scenarioEditor!.getConfig(),
    getRunDurationSec: () => scenarioEditor!.getRunDurationSec(),
    getSimSpeed: () => scenarioEditor!.getSimSpeed(),
  })
  scenarioEditor = new ScenarioEditor(scenarioContainer, {
    onAvailabilityChange: (available) => {
      runControls!.setRunEnabled(available)
      mcResults!.setRunEnabled(available)
    },
  })
}

main().catch((err: unknown) => console.error('OrbitForge init failed:', err))
