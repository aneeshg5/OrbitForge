// Entry point: spawn the simulation worker, wire the WebGL2 scene
// (Earth + orbit paths + covariance ellipsoids) and the Chart.js error/NIS
// panels to the ring buffer, and drive a 60 fps render loop.

import { RingReader } from './bridge/ring_reader.js'
import type { StateFrame } from './bridge/wasm_types.js'
import type { WorkerRequest, WorkerResponse } from './worker.js'
import { EarthRenderer } from './renderer/earth.js'
import { OrbitPathRenderer } from './renderer/orbit.js'
import { CovarianceEllipsoidRenderer } from './renderer/covariance.js'
import { Starfield } from './renderer/starfield.js'
import { PanelManager } from './renderer/panels.js'
import { mat4Identity, mat4LookAt, mat4Perspective, mat4StripTranslation, type Mat4, FILTER_COLOR_RGB } from './renderer/gl_utils.js'
import { ScenarioEditor } from './ui/scenario_editor.js'
import { FaultPanel } from './ui/fault_panel.js'
import { MCResultsPanel } from './ui/mc_results.js'

interface Scene {
  gl: WebGL2RenderingContext
  starfield: Starfield
  earth: EarthRenderer
  orbits: OrbitPathRenderer
  covariances: CovarianceEllipsoidRenderer
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

  const posErrCanvas = document.getElementById('pos-err-canvas') as HTMLCanvasElement
  const velErrCanvas = document.getElementById('vel-err-canvas') as HTMLCanvasElement
  const covTraceCanvas = document.getElementById('cov-trace-canvas') as HTMLCanvasElement
  const nisCanvas = document.getElementById('nis-canvas') as HTMLCanvasElement

  return {
    gl,
    canvas,
    starfield: new Starfield(gl),
    earth: new EarthRenderer(gl),
    orbits: new OrbitPathRenderer(gl),
    covariances: new CovarianceEllipsoidRenderer(gl),
    panels: new PanelManager(posErrCanvas, velErrCanvas, covTraceCanvas, nisCanvas),
  }
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
  const view = camera.viewMatrix()
  const model = mat4Identity()

  // Rendered first so Earth/orbits naturally occlude it via the depth
  // test — see starfield.ts for why a translation-stripped view matrix
  // is what makes it behave as "infinitely far" rather than drifting
  // when the camera zooms.
  scene.starfield.render(mat4StripTranslation(view), proj)

  scene.earth.render(model, view, proj)
  scene.orbits.render(view, proj)

  if (latestFrame) {
    scene.covariances.render(view, proj, latestFrame.kfPos, [latestFrame.kfCovDiag[0], latestFrame.kfCovDiag[1], latestFrame.kfCovDiag[2]], FILTER_COLOR_RGB.kf)
    scene.covariances.render(view, proj, latestFrame.ekfPos, [latestFrame.ekfCovDiag[0], latestFrame.ekfCovDiag[1], latestFrame.ekfCovDiag[2]], FILTER_COLOR_RGB.ekf)
    scene.covariances.render(view, proj, latestFrame.ukfPos, [latestFrame.ukfCovDiag[0], latestFrame.ukfCovDiag[1], latestFrame.ukfCovDiag[2]], FILTER_COLOR_RGB.ukf)
  }

  scene.panels.render()
}

function startRenderLoop(scene: Scene, camera: OrbitCamera, getRingReader: () => RingReader | undefined): void {
  let latestFrame: StateFrame | undefined

  function tick(): void {
    const ringReader = getRingReader()
    if (ringReader) {
      const frames = ringReader.drain()
      for (const frame of frames) {
        feedFrameToScene(scene, frame)
        latestFrame = frame
      }
    }
    camera.update()
    renderScene(scene, camera, latestFrame)
    requestAnimationFrame(tick)
  }
  requestAnimationFrame(tick)
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
  const status = document.getElementById('status')!
  status.textContent = 'loading...'

  registerServiceWorker()

  const scene = setupScene()
  const camera = new OrbitCamera(scene.canvas)

  let ringReader: RingReader | undefined
  startRenderLoop(scene, camera, () => ringReader)

  const worker = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' })
  const postToWorker = (msg: WorkerRequest): void => worker.postMessage(msg)

  const mcContainer = document.getElementById('mc-results-container')!
  const mcResults = new MCResultsPanel(mcContainer, { postToWorker })

  worker.addEventListener('message', (e: MessageEvent<WorkerResponse>) => {
    if (e.data.type === 'ring_buffer_ready') {
      const { sharedArrayBuffer, ringBufferPtr, ringBufferCapacity } = e.data.payload
      ringReader = new RingReader(sharedArrayBuffer, ringBufferPtr, ringBufferCapacity)
      status.textContent = 'running'
    } else if (e.data.type === 'mc_results') {
      mcResults.handleResults(e.data.payload)
    }
  })

  const scenarioContainer = document.getElementById('scenario-editor-container')!
  const faultContainer = document.getElementById('fault-panel-container')!
  new ScenarioEditor(scenarioContainer, { postToWorker })
  new FaultPanel(faultContainer, { postToWorker })

  status.textContent = 'pick a satellite and press Run'
}

main().catch((err: unknown) => console.error('OrbitForge init failed:', err))
