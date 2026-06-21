// Attitude gizmo (Phase 5): a screen-space-fixed orientation indicator in
// the bottom-right corner of the 3D view, showing the spacecraft's true
// body-axes triad (X=red, Y=green, Z=blue). Deliberately NOT rendered as a
// 3D object at the spacecraft's world position — an earlier version did
// that, but at this scene's normal camera distances a triad floating next
// to a tiny orbiting dot was nearly unreadable and moved/scaled with the
// orbit camera, making it hard to use as a stable orientation readout.
//
// This instead uses its own fixed mini-camera (independent of main.ts's
// OrbitCamera entirely) rendered into a small sub-rectangle of the same
// canvas via gl.viewport()/gl.scissor() — the standard "corner axis gizmo"
// pattern from CAD/3D tools (Blender, SolidWorks, etc.), pinned to the
// viewport rather than the 3D world so it stays still and legible
// regardless of how the user has orbited/zoomed the main camera.
//
// No background fill behind the triad (by request) — the lines draw
// directly over whatever the main scene already rendered there. The X/Y/Z
// text labels are HTML, not WebGL, and need to track each rotating axis's
// on-screen tip every frame; computeAxisTipsNdc() exposes the same
// model/view/proj transform render() uses so main.ts can project those
// tips to screen space without duplicating this module's matrix math.
//
// Renders only the true attitude, not three overlapping EKF/UKF/true
// triads — the actual EKF-vs-UKF attitude comparison lives in the chart
// panels (attitude error angle, angular velocity error — panels.ts)
// instead. KF is never shown here — it has no attitude state (§6.1).
//
// AttitudeSmoother (below) caps how fast the *displayed* triad can rotate
// per real second, independent of how fast the true attitude is actually
// tumbling. The satellite's tumble rate (scenario_editor.ts's
// initOmegaZ) is a fixed physical rate, but sim_speed scales how much of
// it plays out per real second — at high sim_speed the raw per-frame
// attitude change can exceed a full rotation between consecutive 60fps
// frames, which strobes/looks like duplicated axes rather than spinning
// (confirmed: at sim_speed=4320 the apparent rate is ~216 rad/s, ~3.6 rad
// of rotation per frame — far past anything a display can show smoothly).
// The underlying physics is untouched; only this gizmo's playback caps at
// a fixed, always-coherent apparent speed.

import { createProgram, mat4LookAt, mat4Multiply, mat4Perspective, type Mat4 } from './gl_utils.js'
import type { QuatCoeffs } from '../bridge/wasm_types.js'

function quatDot(a: QuatCoeffs, b: QuatCoeffs): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

function quatNormalize(q: QuatCoeffs): QuatCoeffs {
  const len = Math.sqrt(quatDot(q, q)) || 1
  return [q[0] / len, q[1] / len, q[2] / len, q[3] / len]
}

// Standard unit-quaternion slerp, t in [0,1]. Takes the shorter arc (flips
// b's sign if the dot product is negative — q and -q represent the same
// orientation, but slerp's geodesic isn't shortest-path-aware on its own).
function quatSlerp(a: QuatCoeffs, b: QuatCoeffs, t: number): QuatCoeffs {
  let dot = quatDot(a, b)
  let bx = b[0], by = b[1], bz = b[2], bw = b[3]
  if (dot < 0) {
    dot = -dot
    bx = -bx; by = -by; bz = -bz; bw = -bw
  }
  if (dot > 0.9995) {
    // Nearly identical orientations — linear interpolation is numerically
    // safer here than the slerp formula below (sinTheta0 -> 0 denominator).
    return quatNormalize([
      a[0] + t * (bx - a[0]),
      a[1] + t * (by - a[1]),
      a[2] + t * (bz - a[2]),
      a[3] + t * (bw - a[3]),
    ])
  }
  const theta0 = Math.acos(dot)
  const theta = theta0 * t
  const sinTheta0 = Math.sin(theta0)
  const s1 = Math.sin(theta) / sinTheta0
  const s0 = Math.cos(theta) - dot * s1
  return [s0 * a[0] + s1 * bx, s0 * a[1] + s1 * by, s0 * a[2] + s1 * bz, s0 * a[3] + s1 * bw]
}

// The actual 3D rotation angle between two unit quaternions (radians) —
// not the same as slerp's internal theta0, which is half this.
function quatAngularDistance(a: QuatCoeffs, b: QuatCoeffs): number {
  return 2 * Math.acos(Math.min(1, Math.abs(quatDot(a, b))))
}

// One full rotation per real second — fast enough to still read as "the
// satellite is actively tumbling," slow enough to stay well clear of
// stroboscopic aliasing at 60fps (6 degrees/frame).
const MAX_GIZMO_ANGULAR_SPEED_RAD_PER_SEC = 2 * Math.PI

/**
 * Tracks a displayed attitude that follows a target quaternion at a capped
 * maximum angular speed, regardless of how large the jump between
 * consecutive target values is. Call update() once per rendered frame
 * (not from both render() and computeAxisTipsNdc() — they must see the
 * same smoothed value within a frame, so main.ts calls this once and
 * passes the result to both).
 */
export class AttitudeSmoother {
  private displayed: QuatCoeffs | undefined
  private lastUpdateMs: number | undefined

  update(target: QuatCoeffs): QuatCoeffs {
    const nowMs = performance.now()
    const dtSec = this.lastUpdateMs !== undefined ? (nowMs - this.lastUpdateMs) / 1000 : 0
    this.lastUpdateMs = nowMs

    if (!this.displayed) {
      this.displayed = target
      return this.displayed
    }

    const maxAngle = MAX_GIZMO_ANGULAR_SPEED_RAD_PER_SEC * dtSec
    const angle = quatAngularDistance(this.displayed, target)
    this.displayed = angle <= maxAngle ? target : quatSlerp(this.displayed, target, maxAngle / angle)
    return this.displayed
  }
}

const VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;

out vec3 v_color;

void main() {
  gl_Position = u_proj * u_view * u_model * vec4(a_position, 1.0);
  v_color = a_color;
}
`

const FRAGMENT_SRC = `#version 300 es
precision highp float;

in vec3 v_color;
out vec4 outColor;

void main() {
  outColor = vec4(v_color, 1.0);
}
`

// Unit-length triad: origin -> +X/+Y/+Z (body frame), colored R/G/B —
// matches the HTML labels main.ts positions at each tip (index.html's
// #axis-label-x/y/z).
const TRIAD_POSITIONS = new Float32Array([
  0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1,
])
const TRIAD_COLORS = new Float32Array([
  1.0, 0.25, 0.25,  1.0, 0.25, 0.25,
  0.3, 0.95, 0.3,   0.3, 0.95, 0.3,
  0.35, 0.55, 1.0,  0.35, 0.55, 1.0,
])

// Fixed mini-camera parameters: an elevated 3/4 view so all three axes are
// distinguishable (a straight-on view down any single axis would
// foreshorten it to a dot) — independent of main.ts's OrbitCamera.
const CAMERA_EYE: [number, number, number] = [1.6, 1.25, 2.0]
const CAMERA_FOV_RAD = (35 * Math.PI) / 180

export interface GizmoViewport {
  x: number
  y: number
  width: number
  height: number
}

/** Normalized device coords (each in [-1,1], y-up) of the three axis tips. */
export interface AxisTipsNdc {
  x: readonly [number, number]
  y: readonly [number, number]
  z: readonly [number, number]
}

interface Matrices {
  model: Mat4
  view: Mat4
  proj: Mat4
}

function buildMatrices(quat: QuatCoeffs, aspect: number): Matrices {
  const [qx, qy, qz, qw] = quat

  // Standard Hamilton-quaternion rotation matrix (body -> ECI) — same
  // convention as Eigen::Quaterniond::toRotationMatrix() on the C++ side.
  const r00 = 1 - 2 * (qy * qy + qz * qz)
  const r01 = 2 * (qx * qy - qz * qw)
  const r02 = 2 * (qx * qz + qy * qw)
  const r10 = 2 * (qx * qy + qz * qw)
  const r11 = 1 - 2 * (qx * qx + qz * qz)
  const r12 = 2 * (qy * qz - qx * qw)
  const r20 = 2 * (qx * qz - qy * qw)
  const r21 = 2 * (qy * qz + qx * qw)
  const r22 = 1 - 2 * (qx * qx + qy * qy)

  // Remap ECI -> scene with the SAME linear map gl_utils.ts's eciToScene
  // applies to ordinary position vectors, so the gizmo's notion of "up"
  // matches the rest of the visualization (scene-Y = ECI-Z = pole) rather
  // than introducing a second, unrelated convention just for this widget.
  // This is a left-multiply M*R: scene_row0 = R_row0, scene_row1 = R_row2,
  // scene_row2 = -R_row1.
  const s00 = r00, s01 = r01, s02 = r02
  const s10 = r20, s11 = r21, s12 = r22
  const s20 = -r10, s21 = -r11, s22 = -r12

  const model: Mat4 = new Float32Array([
    s00, s10, s20, 0,
    s01, s11, s21, 0,
    s02, s12, s22, 0,
    0, 0, 0, 1,
  ])

  const proj = mat4Perspective(CAMERA_FOV_RAD, aspect, 0.1, 10)
  const view = mat4LookAt(CAMERA_EYE, [0, 0, 0], [0, 1, 0])

  return { model, view, proj }
}

/** Column-major mat4 * point(x,y,z,1), returning clip-space (x,y,w). */
function transformPoint(m: Mat4, p: readonly [number, number, number]): readonly [number, number, number] {
  const x = m[0]! * p[0] + m[4]! * p[1] + m[8]! * p[2] + m[12]!
  const y = m[1]! * p[0] + m[5]! * p[1] + m[9]! * p[2] + m[13]!
  const w = m[3]! * p[0] + m[7]! * p[1] + m[11]! * p[2] + m[15]!
  return [x, y, w]
}

export class AttitudeGizmoRenderer {
  private readonly gl: WebGL2RenderingContext
  private readonly program: WebGLProgram
  private readonly vao: WebGLVertexArrayObject
  private readonly uModel: WebGLUniformLocation | null
  private readonly uView: WebGLUniformLocation | null
  private readonly uProj: WebGLUniformLocation | null

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl
    this.program = createProgram(gl, VERTEX_SRC, FRAGMENT_SRC)

    const vao = gl.createVertexArray()
    if (!vao) throw new Error('createVertexArray failed')
    this.vao = vao
    gl.bindVertexArray(vao)

    const posBuf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, posBuf)
    gl.bufferData(gl.ARRAY_BUFFER, TRIAD_POSITIONS, gl.STATIC_DRAW)
    gl.enableVertexAttribArray(0)
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0)

    const colorBuf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuf)
    gl.bufferData(gl.ARRAY_BUFFER, TRIAD_COLORS, gl.STATIC_DRAW)
    gl.enableVertexAttribArray(1)
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0)

    gl.bindVertexArray(null)

    this.uModel = gl.getUniformLocation(this.program, 'u_model')
    this.uView = gl.getUniformLocation(this.program, 'u_view')
    this.uProj = gl.getUniformLocation(this.program, 'u_proj')
  }

  /**
   * quat: body->ECI attitude, Eigen::Quaterniond::coeffs() order (x,y,z,w)
   * — math/quaternion.hpp's convention (StateFrame's trueQuat).
   * viewport: device-pixel rectangle (origin bottom-left, matching
   * WebGL's own viewport convention) to render into — see main.ts for how
   * this is derived from the frame element's on-screen position.
   *
   * Call this LAST in the frame, after the main scene — it overwrites a
   * small sub-rectangle of the same canvas/depth buffer (depth only; no
   * background fill, so the lines draw directly over whatever the main
   * scene already rendered there). Restores the full-canvas viewport
   * before returning so it doesn't leak into the next frame's main-scene
   * draw calls.
   */
  render(quat: QuatCoeffs, viewport: GizmoViewport, canvasWidth: number, canvasHeight: number): void {
    const gl = this.gl
    const aspect = viewport.width / Math.max(1, viewport.height)
    const { model, view, proj } = buildMatrices(quat, aspect)

    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height)
    gl.enable(gl.SCISSOR_TEST)
    gl.scissor(viewport.x, viewport.y, viewport.width, viewport.height)
    gl.clear(gl.DEPTH_BUFFER_BIT) // color NOT cleared — no background, draw directly over the main scene

    gl.useProgram(this.program)
    gl.bindVertexArray(this.vao)
    gl.uniformMatrix4fv(this.uModel, false, model)
    gl.uniformMatrix4fv(this.uView, false, view)
    gl.uniformMatrix4fv(this.uProj, false, proj)
    gl.drawArrays(gl.LINES, 0, 6)
    gl.bindVertexArray(null)

    gl.disable(gl.SCISSOR_TEST)
    gl.viewport(0, 0, canvasWidth, canvasHeight)
  }

  /**
   * Projects the three unit axis tips through the exact same transform
   * render() uses, returning normalized device coordinates (each
   * component in [-1,1], y-up) for main.ts to convert into CSS pixel
   * positions for the X/Y/Z HTML labels. aspect should be viewport.width
   * / viewport.height for the same viewport passed to render().
   */
  computeAxisTipsNdc(quat: QuatCoeffs, aspect: number): AxisTipsNdc {
    const { model, view, proj } = buildMatrices(quat, aspect)
    const mvp = mat4Multiply(proj, mat4Multiply(view, model))
    const toNdc = (p: readonly [number, number, number]): readonly [number, number] => {
      const [x, y, w] = transformPoint(mvp, p)
      const safeW = w === 0 ? 1e-6 : w
      return [x / safeW, y / safeW]
    }
    return {
      x: toNdc([1, 0, 0]),
      y: toNdc([0, 1, 0]),
      z: toNdc([0, 0, 1]),
    }
  }
}
