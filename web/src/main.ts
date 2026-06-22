// Entry point: spawn the simulation worker, wire the WebGL2 scene
// (Earth + orbit paths + covariance ellipsoids) and the Chart.js error/NIS
// panels to the ring buffer, and drive a 60 fps render loop.

import { RingReader } from './bridge/ring_reader.js'
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

// Earth's axial tilt (obliquity) relative to the ecliptic — not modeled
// anywhere in the physics (everything here is ECI, where the spin axis
// defines the Z axis by construction), but baked into the view matrix as
// a fixed world rotation so the globe reads as the familiar tilted Earth
// rather than upright. Applied before the camera transform so it rotates
// Earth, orbit paths, and covariance ellipsoids together consistently.
//
// Earth's mesh (earth.ts buildSphere) and the camera's own orbit
// convention (OrbitCamera orbits around world-Y, i.e. Y is "vertical")
// both already treat Y as the pole/up axis — that pairing is what makes
// the default camera angle read as a normal globe view rather than
// staring down the pole. Rather than fight that by moving Earth's pole to
// match ECI's Z-is-north convention, the ECI->scene remap for satellite
// data lives in orbit.ts/covariance.ts instead (see SCENE_SCALE usage
// there) — so Y stays the on-screen pole and this tilt stays a simple
// roll about the camera's forward (Z) axis, exactly as it always was.
const EARTH_TILT = mat4RotateZ((23.4 * Math.PI) / 180)

// One full rotation per 86400 SIMULATED seconds (one sim-day) — driven by
// simTimeSec, not wall-clock, so it freezes whenever the sim is idle/
// paused exactly like the Sun/Moon already do (solar_system.ts), and a
// completed rotation always means "T+ just passed a day," not a cosmetic
// loop unrelated to the clock. scenario_editor.ts's DEFAULTS.simSpeed is
// set to 86400/20 = 4320 specifically so that, at the default speed, this
// still completes a visible rotation roughly every 20 real seconds — the
// same pacing the old wall-clock-driven version had — instead of being
// imperceptibly slow at 1x. Increasing sim_speed speeds the spin up
// further (and T+ along with it); decreasing it slows both down together.
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

// Arcball-style orbit camera: azimuth/elevation/distance around a fixed
// target, driven by mouse drag with scroll-wheel zoom.
//
// Uses the Pointer Events API + setPointerCapture rather than mousedown/
// mousemove/mouseup: capture keeps receiving move events even if the
// cursor leaves the canvas mid-drag (the old listener attached mousemove
// to the canvas would lose tracking the instant the pointer crossed the
// canvas edge — exactly the kind of thing that reads as "doesn't feel
// normal"). preventDefault() on pointerdown stops the browser's native
// drag-image/text-selection gesture from fighting the rotation, which was
// the dominant cause of the janky feel — without it, every drag was
// racing against the browser's own default click-drag handling.
class OrbitCamera {
  private azimuth = 0.6
  private elevation = 0.4
  private distance = 4.0

  // Angular velocity (rad/ms), decayed each frame for inertia after release.
  private velAzimuth = 0
  private velElevation = 0

  private static readonly k_drag_sensitivity = 0.005
  private static readonly k_damping = 0.92 // per-frame velocity decay once released
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
      // Carry the latest per-event delta as velocity so release feels like
      // a natural throw rather than an instant stop — applied only once
      // dragging stops (see update()), not while still actively dragging.
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

  /** Applies inertial decay to any residual angular velocity from a just-released drag. */
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

// Converts #attitude-gizmo-frame's on-screen CSS position into the device-
// pixel, bottom-left-origin rectangle gl.viewport()/gl.scissor() expect —
// the two coordinate systems disagree on both axis origin (DOM measures
// from the top, WebGL from the bottom) and units (CSS px vs. device px,
// which differ by devicePixelRatio once resizeCanvasToDisplaySize scales
// the canvas's backing buffer). Recomputed every frame rather than cached
// since the frame can move/resize (window resize, sidebar collapse, etc.).
function computeGizmoViewport(canvas: HTMLCanvasElement, frame: HTMLElement): GizmoViewport {
  const canvasRect = canvas.getBoundingClientRect()
  const frameRect = frame.getBoundingClientRect()
  const scaleX = canvas.width / Math.max(1, canvasRect.width)
  const scaleY = canvas.height / Math.max(1, canvasRect.height)

  const width = frameRect.width * scaleX
  const height = frameRect.height * scaleY
  const x = (frameRect.left - canvasRect.left) * scaleX
  const topCss = (frameRect.top - canvasRect.top) * scaleY
  const y = canvas.height - topCss - height // flip: DOM top-origin -> GL bottom-origin

  return { x, y, width, height }
}

// Formats elapsed simulated seconds as "D:HH:MM:SS" (days only shown once
// nonzero — at sim_speed up to 100x a long real-time session can rack up
// many simulated hours quickly) or "HH:MM:SS" otherwise.
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
  const proj = mat4Perspective((45 * Math.PI) / 180, aspect, 0.05, 100)
  const view = mat4Multiply(camera.viewMatrix(), EARTH_TILT)

  // tSec still drives the Sun's surface turbulence/corona wisp shaders
  // (solar_system.ts) continuously, wall-clock as before — those are
  // pure shader liveliness, unrelated to simulated time.
  const tSec = performance.now() / 1000

  // Sun/Moon/Earth-spin are all driven by simulated elapsed time, not
  // wall-clock (see solar_system.ts's SUN_ORBIT_PERIOD_SEC comment for
  // why) — 0 before the sim has produced any frames, so they sit at their
  // initial phase rather than drifting while idle, and freeze whenever the
  // sim is idle/paused without any separate run-state check needed.
  const simTimeSec = latestFrame ? latestFrame.simTime : 0
  scene.simTimeValue.textContent = formatSimTime(simTimeSec)

  const spinAngle = (simTimeSec * SPIN_RAD_PER_SIM_SECOND) % (2 * Math.PI)
  const earthModel = mat4RotateY(spinAngle)

  // Current Sun direction (Earth -> Sun) — see solar_system.ts's
  // sunDirectionScene(). Used as the light source for both Earth and the
  // Moon, so the visible Sun position and the lit hemisphere always stay
  // in sync as the Sun orbits.
  const sunDirScene = sunDirectionScene(simTimeSec)

  // Shared by the satellite marker and the attitude gizmo below — both
  // must render the same smoothed orientation within one frame (see
  // AttitudeSmoother's doc in attitude.ts), so this is computed once here
  // rather than letting each call site invoke update() independently.
  const attitudeTargetQuat: StateFrame['trueQuat'] = latestFrame ? latestFrame.trueQuat : [0, 0, 0, 1]
  const smoothedQuat = scene.attitudeSmoother.update(attitudeTargetQuat)

  // Starfield still uses a translation-stripped view (genuinely
  // infinitely distant) — see starfield.ts. The Sun and Moon are real,
  // near scene objects now (see solar_system.ts's module comment) and
  // use the ordinary view matrix below.
  const viewRotation = mat4StripTranslation(view)
  scene.starfield.render(viewRotation, proj)

  scene.earth.render(earthModel, view, proj, sunDirScene)
  scene.axis.render(view, proj)
  scene.orbits.render(view, proj)

  if (latestFrame) {
    // KF's cov diag is [r(3),v(3)] — position variance stays at [0..2].
    // EKF/UKF's is Phase 5's 12-state [delta_theta(3),omega(3),r(3),v(3)]
    // — position variance moved to [6..8] (index [0..2] is now the
    // attitude-error variance, a different physical quantity entirely).
    scene.covariances.render(view, proj, latestFrame.kfPos, [latestFrame.kfCovDiag[0], latestFrame.kfCovDiag[1], latestFrame.kfCovDiag[2]], FILTER_COLOR_RGB.kf)
    scene.covariances.render(view, proj, latestFrame.ekfPos, [latestFrame.ekfCovDiag[6], latestFrame.ekfCovDiag[7], latestFrame.ekfCovDiag[8]], FILTER_COLOR_RGB.ekf)
    scene.covariances.render(view, proj, latestFrame.ukfPos, [latestFrame.ukfCovDiag[6], latestFrame.ukfCovDiag[7], latestFrame.ukfCovDiag[8]], FILTER_COLOR_RGB.ukf)

    // The actual spacecraft, at its true position/attitude — see
    // satellite_marker.ts's module doc. Only once a frame exists; there's
    // nothing to place at the origin's identity attitude before Run.
    scene.satelliteMarker.render(view, proj, latestFrame.truePos, smoothedQuat, sunDirScene)
  }

  scene.solarSystem.render(view, proj, sunDirScene, tSec, simTimeSec)

  // Phase 5: fixed-corner attitude gizmo — see attitude.ts's doc for why
  // this renders only the true attitude (not EKF/UKF too) as a
  // screen-space-fixed indicator rather than a 3D object in the world.
  // Renders unconditionally, not just once latestFrame exists — an
  // identity-quaternion fallback before the sim has produced any frames,
  // so the gizmo is never an empty box waiting for Run to be clicked.
  // Must be the LAST draw of the frame: it overwrites a small
  // sub-rectangle of this same canvas/depth buffer, so anything drawn
  // after it (there is nothing) would paint over it.
  const gizmoViewport = computeGizmoViewport(scene.canvas, scene.attitudeGizmoFrame)
  const gizmoQuat = smoothedQuat
  scene.attitude.render(gizmoQuat, gizmoViewport, canvas.width, canvas.height)

  // X/Y/Z labels track their actual rotating axis tip rather than sitting
  // in a static legend — project each tip through the same transform
  // render() just used, then convert from NDC to a CSS pixel position
  // relative to #scene-container (the labels' positioned ancestor).
  const gizmoAspect = gizmoViewport.width / Math.max(1, gizmoViewport.height)
  const axisTips = scene.attitude.computeAxisTipsNdc(gizmoQuat, gizmoAspect)
  const frameRect = scene.attitudeGizmoFrame.getBoundingClientRect()
  const containerRect = canvas.parentElement!.getBoundingClientRect()
  positionAxisLabel(scene.axisLabels.x, axisTips.x, frameRect, containerRect)
  positionAxisLabel(scene.axisLabels.y, axisTips.y, frameRect, containerRect)
  positionAxisLabel(scene.axisLabels.z, axisTips.z, frameRect, containerRect)

  scene.panels.render()
}

// ndc: normalized device coords ([-1,1], y-up) of a point within
// frameRect (the gizmo's on-screen CSS rect). Converts to a CSS pixel
// position relative to containerRect (#scene-container, the labels'
// nearest positioned ancestor) and applies it directly to the element.
function positionAxisLabel(
  label: HTMLElement,
  ndc: readonly [number, number],
  frameRect: DOMRect,
  containerRect: DOMRect,
): void {
  const xWithinFrame = (ndc[0] * 0.5 + 0.5) * frameRect.width
  const yWithinFrame = (1 - (ndc[1] * 0.5 + 0.5)) * frameRect.height // flip: NDC y-up -> CSS y-down
  label.style.left = `${frameRect.left - containerRect.left + xWithinFrame}px`
  label.style.top = `${frameRect.top - containerRect.top + yWithinFrame}px`
}

// Returns a resetView() handle so main() can clear the previous run's
// stale state client-side the moment Reset is clicked — independent of the
// worker roundtrip, so the old orbit trail/satellite position don't linger
// on screen until the next frame happens to arrive (it may never arrive at
// all if the user doesn't press Run again right away).
function startRenderLoop(
  scene: Scene,
  camera: OrbitCamera,
  getRingReader: () => RingReader | undefined,
  getRunControls: () => RunControls | undefined,
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
      // Authoritative: fires once the ring buffer is observed to have
      // actually wrapped (RingReader.drain()'s doc) — covers the gap
      // between resetView()'s optimistic clear and the worker really
      // having run reset_simulation(), during which a frame or two from
      // just before the reset could otherwise still land and make T+
      // briefly show a stale non-zero value after Reset.
      if (reset) clearView()
      for (const frame of frames) {
        feedFrameToScene(scene, frame)
        latestFrame = frame
      }
    }
    camera.update()
    renderScene(scene, camera, latestFrame)
    getRunControls()?.checkAutoStop(latestFrame ? latestFrame.simTime : 0)
    requestAnimationFrame(tick)
  }
  requestAnimationFrame(tick)

  return { resetView: clearView, getCurrentSimTimeSec: () => (latestFrame ? latestFrame.simTime : 0) }
}

function registerServiceWorker(): void {
  // Skipped in dev: vite's dev server serves modules individually with no
  // build step, so the SW's cache-first strategy (sw.ts) masks source
  // edits behind a stale cache instead of reflecting them. A registration
  // from a previous session can also linger across reloads — clear
  // navigator.serviceWorker.getRegistrations() and caches.keys() if a fix
  // to web code doesn't seem to take effect in dev.
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

  // runControls is constructed later (it needs scenarioEditor, see below) —
  // referenced here only inside a closure invoked every render frame, by
  // which point it's assigned. Mirrors the ringReader forward-reference
  // pattern just below.
  let runControls: RunControls | undefined
  let ringReader: RingReader | undefined
  const { resetView, getCurrentSimTimeSec } = startRenderLoop(scene, camera, () => ringReader, () => runControls)

  const worker = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' })
  const postToWorker = (msg: WorkerRequest): void => {
    if (msg.type === 'reset') resetView()
    worker.postMessage(msg)
  }

  const mcContainer = document.getElementById('mc-results-container')!
  const mcResults = new MCResultsPanel(mcContainer, { postToWorker })

  worker.addEventListener('message', (e: MessageEvent<WorkerResponse>) => {
    if (e.data.type === 'ring_buffer_ready') {
      const { sharedArrayBuffer, ringBufferPtr, ringBufferCapacity } = e.data.payload
      ringReader = new RingReader(sharedArrayBuffer, ringBufferPtr, ringBufferCapacity)
    } else if (e.data.type === 'mc_results') {
      mcResults.handleResults(e.data.payload)
    }
  })

  const scenarioContainer = document.getElementById('scenario-editor-container')!
  const faultContainer = document.getElementById('fault-panel-container')!
  new FaultPanel(faultContainer, { postToWorker, getCurrentSimTimeSec })

  // RunControls.getConfig needs scenarioEditor and scenarioEditor's
  // availability callback needs runControls — constructed in this order
  // (runControls first, referencing scenarioEditor only inside a closure
  // invoked later on click) to break the cycle without a null check.
  let scenarioEditor: ScenarioEditor
  const runControlsContainer = document.getElementById('run-controls-container')!
  runControls = new RunControls(runControlsContainer, {
    postToWorker,
    getConfig: () => scenarioEditor.getConfig(),
    getRunDurationSec: () => scenarioEditor.getRunDurationSec(),
    getSimSpeed: () => scenarioEditor.getSimSpeed(),
  })
  scenarioEditor = new ScenarioEditor(scenarioContainer, {
    onAvailabilityChange: (available) => runControls!.setRunEnabled(available),
  })
}

main().catch((err: unknown) => console.error('OrbitForge init failed:', err))
