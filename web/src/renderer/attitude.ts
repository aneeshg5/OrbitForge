// Body-axes attitude triad (Phase 5): three colored lines (X=red, Y=green,
// Z=blue — the standard body-axes convention) at the spacecraft's true
// position, oriented by the true attitude quaternion. Gives the orbit a
// tangible "this is the actual spacecraft, and it's tumbling" payoff,
// which nothing in the scene rendered before this (the orbit path is just
// a line; covariance.ts draws ellipsoids at the filter estimates, not the
// true position).
//
// Deliberately renders ONLY the true attitude, not three overlapping
// triads for true/EKF/UKF — at the camera distances this scene normally
// sits at, three triads near the same point would be unreadable clutter,
// not a useful comparison. The actual EKF-vs-UKF attitude-estimation
// comparison lives in the new chart panels (attitude error angle, angular
// velocity error — panels.ts) instead, which is what those numbers are
// actually for. KF is never rendered here — it has no attitude state at
// all (§6.1) — and callers (main.ts) should not call render() for it.

import { createProgram, eciToScene, type Mat4, SCENE_SCALE } from './gl_utils.js'
import type { QuatCoeffs } from '../bridge/wasm_types.js'

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

// Unit-length triad: origin -> +X/+Y/+Z (body frame), colored R/G/B.
// Scaled by ARM_LENGTH_SCENE in the model matrix below.
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

// Scene units — larger than the Moon's mesh radius (0.27, solar_system.ts)
// so the triad reads clearly against the orbit-scale view this renders at.
const ARM_LENGTH_SCENE = 0.4

export class AttitudeTriadRenderer {
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
   * centerEci: spacecraft position [x,y,z] in ECI meters (StateFrame's
   * truePos). quat: body->ECI attitude, Eigen::Quaterniond::coeffs() order
   * (x,y,z,w) — math/quaternion.hpp's convention (StateFrame's trueQuat).
   */
  render(view: Mat4, proj: Mat4, centerEci: readonly [number, number, number], quat: QuatCoeffs): void {
    const gl = this.gl
    const [qx, qy, qz, qw] = quat

    // Standard Hamilton-quaternion rotation matrix (body -> ECI) — the
    // same convention as Eigen::Quaterniond::toRotationMatrix() on the
    // C++ side, not independently re-derived here.
    const r00 = 1 - 2 * (qy * qy + qz * qz)
    const r01 = 2 * (qx * qy - qz * qw)
    const r02 = 2 * (qx * qz + qy * qw)
    const r10 = 2 * (qx * qy + qz * qw)
    const r11 = 1 - 2 * (qx * qx + qz * qz)
    const r12 = 2 * (qy * qz - qx * qw)
    const r20 = 2 * (qx * qz - qy * qw)
    const r21 = 2 * (qy * qz + qx * qw)
    const r22 = 1 - 2 * (qx * qx + qy * qy)

    // Remap ECI -> scene with the SAME linear map gl_utils.ts's
    // eciToScene applies to ordinary position vectors ((x,y,z)->(x,z,-y)),
    // applied here to R's rows since this is a left-multiply M*R:
    // scene_row0 = R_row0, scene_row1 = R_row2, scene_row2 = -R_row1.
    const s00 = r00, s01 = r01, s02 = r02
    const s10 = r20, s11 = r21, s12 = r22
    const s20 = -r10, s21 = -r11, s22 = -r12

    const center = eciToScene(centerEci)
    const model = new Float32Array([
      s00 * ARM_LENGTH_SCENE, s10 * ARM_LENGTH_SCENE, s20 * ARM_LENGTH_SCENE, 0,
      s01 * ARM_LENGTH_SCENE, s11 * ARM_LENGTH_SCENE, s21 * ARM_LENGTH_SCENE, 0,
      s02 * ARM_LENGTH_SCENE, s12 * ARM_LENGTH_SCENE, s22 * ARM_LENGTH_SCENE, 0,
      center[0] * SCENE_SCALE, center[1] * SCENE_SCALE, center[2] * SCENE_SCALE, 1,
    ])

    gl.useProgram(this.program)
    gl.bindVertexArray(this.vao)
    gl.uniformMatrix4fv(this.uModel, false, model)
    gl.uniformMatrix4fv(this.uView, false, view)
    gl.uniformMatrix4fv(this.uProj, false, proj)
    gl.drawArrays(gl.LINES, 0, 6)
    gl.bindVertexArray(null)
  }
}
