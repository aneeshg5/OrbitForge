// Orbit path renderer: one GL_LINE_STRIP per trajectory (true, KF, EKF,
// UKF), built up from the ECI positions streamed in each StateFrame.
// Positions arrive in meters; SCENE_SCALE converts to the same scene units
// earth.ts's unit sphere uses.

import { createProgram, eciToScene, type Mat4, SCENE_SCALE, FILTER_COLOR_RGB } from './gl_utils.js'

const VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;

void main() {
  gl_Position = u_proj * u_view * vec4(a_position, 1.0);
}
`

// The true path is rendered solid white rather than dashed — a real dash
// pattern needs a per-vertex "distance along path" attribute and
// discard-based stippling in the fragment shader, which adds real
// complexity for a cosmetic distinction already covered by the true
// path's distinct (white vs. blue/teal/orange) color.
const FRAGMENT_SRC = `#version 300 es
precision highp float;

uniform vec3 u_color;
out vec4 outColor;

void main() {
  outColor = vec4(u_color, 1.0);
}
`

export type PathKind = 'true' | 'kf' | 'ekf' | 'ukf'

const PATH_COLORS: Record<PathKind, readonly [number, number, number]> = {
  true: [1.0, 1.0, 1.0],
  kf: FILTER_COLOR_RGB.kf,
  ekf: FILTER_COLOR_RGB.ekf,
  ukf: FILTER_COLOR_RGB.ukf,
}

const MAX_POINTS_PER_PATH = 2048

class Path {
  readonly buf: Float32Array
  count = 0
  private writeIdx = 0
  private readonly glBuf: WebGLBuffer
  private dirty = false

  constructor(private readonly gl: WebGL2RenderingContext) {
    this.buf = new Float32Array(MAX_POINTS_PER_PATH * 3)
    const b = gl.createBuffer()
    if (!b) throw new Error('createBuffer failed')
    this.glBuf = b
    gl.bindBuffer(gl.ARRAY_BUFFER, this.glBuf)
    gl.bufferData(gl.ARRAY_BUFFER, this.buf.byteLength, gl.DYNAMIC_DRAW)
  }

  addPoint(x: number, y: number, z: number): void {
    if (this.count < MAX_POINTS_PER_PATH) {
      this.writeIdx = this.count
      this.count++
    } else {
      // Buffer full: shift the window forward by dropping the oldest point.
      // Simple ring-less compaction — orbit paths are visual history, not a
      // performance-critical hot path, so an O(n) memmove per overflow is fine.
      this.buf.copyWithin(0, 3, this.buf.length)
      this.writeIdx = this.count - 1
    }
    const i = this.writeIdx * 3
    this.buf[i] = x
    this.buf[i + 1] = y
    this.buf[i + 2] = z
    this.dirty = true
  }

  upload(): void {
    if (!this.dirty) return
    const gl = this.gl
    gl.bindBuffer(gl.ARRAY_BUFFER, this.glBuf)
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, this.buf.subarray(0, this.count * 3))
    this.dirty = false
  }

  bind(): void {
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.glBuf)
    this.gl.enableVertexAttribArray(0)
    this.gl.vertexAttribPointer(0, 3, this.gl.FLOAT, false, 0, 0)
  }
}

export class OrbitPathRenderer {
  private readonly gl: WebGL2RenderingContext
  private readonly program: WebGLProgram
  private readonly vao: WebGLVertexArrayObject
  private readonly paths: Record<PathKind, Path>
  private readonly uView: WebGLUniformLocation | null
  private readonly uProj: WebGLUniformLocation | null
  private readonly uColor: WebGLUniformLocation | null

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl
    this.program = createProgram(gl, VERTEX_SRC, FRAGMENT_SRC)

    const vao = gl.createVertexArray()
    if (!vao) throw new Error('createVertexArray failed')
    this.vao = vao

    this.paths = {
      true: new Path(gl),
      kf: new Path(gl),
      ekf: new Path(gl),
      ukf: new Path(gl),
    }

    this.uView = gl.getUniformLocation(this.program, 'u_view')
    this.uProj = gl.getUniformLocation(this.program, 'u_proj')
    this.uColor = gl.getUniformLocation(this.program, 'u_color')
  }

  /** position is ECI meters [x, y, z] — see gl_utils.eciToScene for the axis remap. */
  addPoint(kind: PathKind, position: readonly [number, number, number]): void {
    const [x, y, z] = eciToScene(position)
    this.paths[kind].addPoint(x * SCENE_SCALE, y * SCENE_SCALE, z * SCENE_SCALE)
  }

  clear(): void {
    for (const kind of Object.keys(this.paths) as PathKind[]) {
      this.paths[kind].count = 0
    }
  }

  render(view: Mat4, proj: Mat4): void {
    const gl = this.gl
    gl.useProgram(this.program)
    gl.bindVertexArray(this.vao)
    gl.uniformMatrix4fv(this.uView, false, view)
    gl.uniformMatrix4fv(this.uProj, false, proj)

    for (const kind of Object.keys(this.paths) as PathKind[]) {
      const path = this.paths[kind]
      if (path.count < 2) continue
      path.upload()
      path.bind()
      const [r, g, b] = PATH_COLORS[kind]
      gl.uniform3f(this.uColor, r, g, b)
      gl.drawArrays(gl.LINE_STRIP, 0, path.count)
    }
    gl.bindVertexArray(null)
  }
}
