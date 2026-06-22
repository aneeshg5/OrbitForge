// A small shaded sphere at the spacecraft's true ECI position, oriented by
// its true attitude — the actual "what does the satellite look like right
// now" object the rest of the scene (orbit paths, covariance ellipsoids,
// the attitude gizmo) was always missing: everything else shows a derived
// quantity (a path, an uncertainty bound, a fixed-corner orientation
// readout), nothing shows the spacecraft itself moving and tumbling in the
// world. Renders only the true state, matching attitude.ts's precedent of
// not overlaying KF/EKF/UKF attitudes in the 3D view (the chart panels are
// where filter comparison actually happens).
//
// Two visual cues make the tumble readable, since a perfectly uniform red
// ball spinning in place is indistinguishable from a stationary one:
//   1. A body-fixed dark/light color split (by local +X vs -X) — this
//      boundary is baked into object space, so it visibly sweeps across
//      the sphere as u_model's rotation changes, independent of lighting.
//   2. Lambertian shading from the same sun direction main.ts already
//      computes for Earth/Moon (solar_system.ts's sunDirectionScene) — a
//      secondary depth cue, not the primary one, since the Sun moves far
//      slower than most tumble rates and would alone look almost frozen.
//
// Reuses main.ts's AttitudeSmoother-capped quaternion (the same one passed
// to attitude.ts's gizmo) rather than the raw trueQuat — at high sim_speed
// the raw per-frame attitude delta can exceed a full rotation between
// consecutive 60fps frames (see attitude.ts's module comment), which would
// strobe this marker exactly as it would the gizmo. Both consumers must
// share one smoothed value per frame, not run independent smoothers.

import { createProgram, eciToScene, type Mat4, SCENE_SCALE } from './gl_utils.js'
import { buildSphere } from './earth.js'
import type { QuatCoeffs } from '../bridge/wasm_types.js'

const VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

out vec3 v_normalWorld;
out vec3 v_localPos;

void main() {
  gl_Position = u_proj * u_view * u_model * vec4(a_position, 1.0);
  v_normalWorld = mat3(u_model) * a_normal;
  v_localPos = a_position;
}
`

const FRAGMENT_SRC = `#version 300 es
precision highp float;

uniform vec3 u_sunDirWorld;

in vec3 v_normalWorld;
in vec3 v_localPos;

out vec4 outColor;

void main() {
  vec3 n = normalize(v_normalWorld);

  // Body-fixed hemisphere split (local +X bright, -X dark) — the primary
  // spin cue, see module comment.
  float side = step(0.0, v_localPos.x);
  vec3 bright = vec3(0.95, 0.18, 0.16);
  vec3 dark = vec3(0.26, 0.04, 0.04);
  vec3 base = mix(dark, bright, side);

  float ndotl = max(dot(n, normalize(u_sunDirWorld)), 0.0);
  float shade = 0.35 + 0.65 * ndotl;
  outColor = vec4(base * shade, 1.0);
}
`

// Stylized, not to scale (real spacecraft are far smaller than this
// fraction of Earth's radius) — same tradeoff orbit.ts's line width and
// covariance.ts's ellipsoid scale already make: too small to see at this
// camera's normal zoom range otherwise.
const MARKER_RADIUS_SCENE_UNITS = 0.02

export class SatelliteMarkerRenderer {
  private readonly gl: WebGL2RenderingContext
  private readonly program: WebGLProgram
  private readonly vao: WebGLVertexArrayObject
  private readonly indexCount: number
  private readonly indexType: number
  private readonly uModel: WebGLUniformLocation | null
  private readonly uView: WebGLUniformLocation | null
  private readonly uProj: WebGLUniformLocation | null
  private readonly uSunDirWorld: WebGLUniformLocation | null

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl
    this.program = createProgram(gl, VERTEX_SRC, FRAGMENT_SRC)

    const geom = buildSphere(1.0, 10, 14)
    this.indexCount = geom.indices.length
    this.indexType = geom.indices instanceof Uint32Array ? gl.UNSIGNED_INT : gl.UNSIGNED_SHORT

    const vao = gl.createVertexArray()
    if (!vao) throw new Error('createVertexArray failed')
    this.vao = vao
    gl.bindVertexArray(vao)

    const posBuf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, posBuf)
    gl.bufferData(gl.ARRAY_BUFFER, geom.positions, gl.STATIC_DRAW)
    gl.enableVertexAttribArray(0)
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0)

    const normalBuf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, normalBuf)
    gl.bufferData(gl.ARRAY_BUFFER, geom.normals, gl.STATIC_DRAW)
    gl.enableVertexAttribArray(1)
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0)

    const indexBuf = gl.createBuffer()
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indexBuf)
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, geom.indices, gl.STATIC_DRAW)
    gl.bindVertexArray(null)

    this.uModel = gl.getUniformLocation(this.program, 'u_model')
    this.uView = gl.getUniformLocation(this.program, 'u_view')
    this.uProj = gl.getUniformLocation(this.program, 'u_proj')
    this.uSunDirWorld = gl.getUniformLocation(this.program, 'u_sunDirWorld')
  }

  /**
   * posEci: true position (StateFrame.truePos), ECI meters.
   * quat: true attitude, body->ECI, Eigen coeffs() order (x,y,z,w) — pass
   * main.ts's AttitudeSmoother-capped value (same one given to the
   * attitude gizmo), not the raw per-frame trueQuat (see module comment).
   * sunDirWorld: scene-space Earth->Sun unit vector (solar_system.ts's
   * sunDirectionScene), shared with earth.ts's lighting for consistency.
   */
  render(
    view: Mat4,
    proj: Mat4,
    posEci: readonly [number, number, number],
    quat: QuatCoeffs,
    sunDirWorld: readonly [number, number, number],
  ): void {
    const gl = this.gl
    const [qx, qy, qz, qw] = quat

    // Same Hamilton rotation-matrix formula as attitude.ts's buildMatrices,
    // remapped from ECI to scene axes identically (scene_row0 = R_row0,
    // scene_row1 = R_row2, scene_row2 = -R_row1) so this marker's body
    // orientation agrees with the gizmo's.
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

    const center = eciToScene(posEci)
    const radius = MARKER_RADIUS_SCENE_UNITS

    const model: Mat4 = new Float32Array([
      radius * s00, radius * s10, radius * s20, 0,
      radius * s01, radius * s11, radius * s21, 0,
      radius * s02, radius * s12, radius * s22, 0,
      center[0] * SCENE_SCALE, center[1] * SCENE_SCALE, center[2] * SCENE_SCALE, 1,
    ])

    gl.enable(gl.DEPTH_TEST)
    gl.useProgram(this.program)
    gl.bindVertexArray(this.vao)
    gl.uniformMatrix4fv(this.uModel, false, model)
    gl.uniformMatrix4fv(this.uView, false, view)
    gl.uniformMatrix4fv(this.uProj, false, proj)
    gl.uniform3f(this.uSunDirWorld, sunDirWorld[0], sunDirWorld[1], sunDirWorld[2])
    gl.drawElements(gl.TRIANGLES, this.indexCount, this.indexType, 0)
    gl.bindVertexArray(null)
  }
}
