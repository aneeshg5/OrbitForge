import { createProgram, mat4LookAt, mat4Multiply, mat4Perspective, type Mat4 } from './gl_utils.js'
import type { QuatCoeffs } from '../bridge/wasm_types.js'

function quatDot(a: QuatCoeffs, b: QuatCoeffs): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

function quatNormalize(q: QuatCoeffs): QuatCoeffs {
  const len = Math.sqrt(quatDot(q, q)) || 1
  return [q[0] / len, q[1] / len, q[2] / len, q[3] / len]
}

function quatSlerp(a: QuatCoeffs, b: QuatCoeffs, t: number): QuatCoeffs {
  let dot = quatDot(a, b)
  let bx = b[0], by = b[1], bz = b[2], bw = b[3]
  if (dot < 0) {
    dot = -dot
    bx = -bx; by = -by; bz = -bz; bw = -bw
  }
  if (dot > 0.9995) {
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

function quatAngularDistance(a: QuatCoeffs, b: QuatCoeffs): number {
  return 2 * Math.acos(Math.min(1, Math.abs(quatDot(a, b))))
}

const MAX_GIZMO_ANGULAR_SPEED_RAD_PER_SEC = 2 * Math.PI

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

const CAMERA_EYE: [number, number, number] = [1.6, 1.25, 2.0]
const CAMERA_FOV_RAD = (35 * Math.PI) / 180

export interface GizmoViewport {
  x: number
  y: number
  width: number
  height: number
}

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

  const r00 = 1 - 2 * (qy * qy + qz * qz)
  const r01 = 2 * (qx * qy - qz * qw)
  const r02 = 2 * (qx * qz + qy * qw)
  const r10 = 2 * (qx * qy + qz * qw)
  const r11 = 1 - 2 * (qx * qx + qz * qz)
  const r12 = 2 * (qy * qz - qx * qw)
  const r20 = 2 * (qx * qz - qy * qw)
  const r21 = 2 * (qy * qz + qx * qw)
  const r22 = 1 - 2 * (qx * qx + qy * qy)

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

  render(quat: QuatCoeffs, viewport: GizmoViewport, canvasWidth: number, canvasHeight: number): void {
    const gl = this.gl
    const aspect = viewport.width / Math.max(1, viewport.height)
    const { model, view, proj } = buildMatrices(quat, aspect)

    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height)
    gl.enable(gl.SCISSOR_TEST)
    gl.scissor(viewport.x, viewport.y, viewport.width, viewport.height)
    gl.clear(gl.DEPTH_BUFFER_BIT)

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
