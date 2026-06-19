// 3-sigma covariance ellipsoid wireframe.
//
// The general approach is to eigendecompose P[0:3,0:3] into 3 semi-axes
// and transform a unit sphere accordingly. That eigendecomposition needs
// the full 3x3 position covariance block, including off-diagonal terms —
// but StateFrame (engine/include/wasm_api.hpp) only transmits the
// covariance *diagonal* (kf_cov_diag etc., 6 doubles: 3 position + 3
// velocity variances), a bandwidth/simplicity decision. A diagonal
// matrix's eigenvectors are exactly the coordinate axes and its
// eigenvalues are its diagonal entries, so what's actually rendered here
// is an ECI-axis-aligned ellipsoid with semi-axes 3*sqrt(variance) per
// axis, not a body-frame-oriented one. This is a real simplification, not
// a bug — it would need StateFrame to carry off-diagonal covariance terms
// to do the general case, which isn't currently planned.

import { createProgram, type Mat4, SCENE_SCALE } from './gl_utils.js'

const VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;

void main() {
  gl_Position = u_proj * u_view * u_model * vec4(a_position, 1.0);
}
`

const FRAGMENT_SRC = `#version 300 es
precision highp float;
uniform vec3 u_color;
out vec4 outColor;
void main() {
  outColor = vec4(u_color, 0.6);
}
`

// Wireframe unit sphere: a small number of latitude + longitude rings,
// rendered as LINE_LOOP per ring — enough to read as an ellipsoid without
// the vertex count of a full shaded sphere.
function buildWireSphereRings(): { positions: Float32Array; ringStarts: number[]; ringLengths: number[] } {
  const segmentsPerRing = 48
  const positions: number[] = []
  const ringStarts: number[] = []
  const ringLengths: number[] = []

  // 3 latitude rings (around the Z axis, i.e. horizontal circles at fixed y)
  const latFractions = [-0.5, 0.0, 0.5]
  for (const f of latFractions) {
    const y = f * 2 // covers roughly the upper/lower bands of the sphere
    const r = Math.sqrt(Math.max(0, 1 - Math.min(1, y * y)))
    ringStarts.push(positions.length / 3)
    for (let i = 0; i < segmentsPerRing; i++) {
      const theta = (i / segmentsPerRing) * 2 * Math.PI
      positions.push(r * Math.cos(theta), y, r * Math.sin(theta))
    }
    ringLengths.push(segmentsPerRing)
  }

  // 3 "longitude" rings (great circles through the poles, rotated about Y)
  for (let ring = 0; ring < 3; ring++) {
    const rotY = (ring / 3) * Math.PI
    ringStarts.push(positions.length / 3)
    for (let i = 0; i < segmentsPerRing; i++) {
      const theta = (i / segmentsPerRing) * 2 * Math.PI
      const x0 = Math.cos(theta)
      const y0 = Math.sin(theta)
      const x = x0 * Math.cos(rotY)
      const z = x0 * Math.sin(rotY)
      positions.push(x, y0, z)
    }
    ringLengths.push(segmentsPerRing)
  }

  return { positions: new Float32Array(positions), ringStarts, ringLengths }
}

export class CovarianceEllipsoidRenderer {
  private readonly gl: WebGL2RenderingContext
  private readonly program: WebGLProgram
  private readonly vao: WebGLVertexArrayObject
  private readonly ringStarts: number[]
  private readonly ringLengths: number[]
  private readonly uModel: WebGLUniformLocation | null
  private readonly uView: WebGLUniformLocation | null
  private readonly uProj: WebGLUniformLocation | null
  private readonly uColor: WebGLUniformLocation | null

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl
    this.program = createProgram(gl, VERTEX_SRC, FRAGMENT_SRC)

    const { positions, ringStarts, ringLengths } = buildWireSphereRings()
    this.ringStarts = ringStarts
    this.ringLengths = ringLengths

    const vao = gl.createVertexArray()
    if (!vao) throw new Error('createVertexArray failed')
    this.vao = vao
    gl.bindVertexArray(vao)
    const buf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, buf)
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW)
    gl.enableVertexAttribArray(0)
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0)
    gl.bindVertexArray(null)

    this.uModel = gl.getUniformLocation(this.program, 'u_model')
    this.uView = gl.getUniformLocation(this.program, 'u_view')
    this.uProj = gl.getUniformLocation(this.program, 'u_proj')
    this.uColor = gl.getUniformLocation(this.program, 'u_color')
  }

  /**
   * centerEci: filter position estimate [x,y,z] in ECI meters.
   * posVarianceDiag: [var_x, var_y, var_z] in m^2 (StateFrame's
   * *_cov_diag[0..2]).
   *
   * Same (x,y,z) -> (x,z,-y) remap as orbit.ts (see its addPoint() for
   * why) applied to both the center and the per-axis scale — the variance
   * is diagonal/axis-aligned in ECI, so remapping which scene axis each
   * sigma lands on keeps the ellipsoid aligned with the remapped position
   * data instead of pointing the wrong way relative to it.
   */
  render(
    view: Mat4,
    proj: Mat4,
    centerEci: readonly [number, number, number],
    posVarianceDiag: readonly [number, number, number],
    color: readonly [number, number, number],
  ): void {
    const gl = this.gl
    const sigma3 = posVarianceDiag.map((v) => 3 * Math.sqrt(Math.max(0, v)) * SCENE_SCALE)

    const model = new Float32Array([
      sigma3[0]!, 0, 0, 0,
      0, sigma3[2]!, 0, 0,
      0, 0, sigma3[1]!, 0,
      centerEci[0] * SCENE_SCALE, centerEci[2] * SCENE_SCALE, -centerEci[1] * SCENE_SCALE, 1,
    ])

    gl.useProgram(this.program)
    gl.bindVertexArray(this.vao)
    gl.uniformMatrix4fv(this.uModel, false, model)
    gl.uniformMatrix4fv(this.uView, false, view)
    gl.uniformMatrix4fv(this.uProj, false, proj)
    gl.uniform3f(this.uColor, color[0], color[1], color[2])

    gl.enable(gl.BLEND)
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
    for (let i = 0; i < this.ringStarts.length; i++) {
      gl.drawArrays(gl.LINE_LOOP, this.ringStarts[i]!, this.ringLengths[i]!)
    }
    gl.disable(gl.BLEND)
    gl.bindVertexArray(null)
  }
}
