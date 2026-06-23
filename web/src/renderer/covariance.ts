import { createProgram, eciToScene, type Mat4, SCENE_SCALE } from './gl_utils.js'

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

function buildWireSphereRings(): { positions: Float32Array; ringStarts: number[]; ringLengths: number[] } {
  const segmentsPerRing = 48
  const positions: number[] = []
  const ringStarts: number[] = []
  const ringLengths: number[] = []

  const latFractions = [-0.5, 0.0, 0.5]
  for (const f of latFractions) {
    const y = f * 2
    const r = Math.sqrt(Math.max(0, 1 - Math.min(1, y * y)))
    ringStarts.push(positions.length / 3)
    for (let i = 0; i < segmentsPerRing; i++) {
      const theta = (i / segmentsPerRing) * 2 * Math.PI
      positions.push(r * Math.cos(theta), y, r * Math.sin(theta))
    }
    ringLengths.push(segmentsPerRing)
  }

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

  render(
    view: Mat4,
    proj: Mat4,
    centerEci: readonly [number, number, number],
    posVarianceDiag: readonly [number, number, number],
    color: readonly [number, number, number],
  ): void {
    const gl = this.gl
    const sigma = (v: number): number => 3 * Math.sqrt(Math.max(0, v)) * SCENE_SCALE
    const center = eciToScene(centerEci)

    const model = new Float32Array([
      sigma(posVarianceDiag[0]), 0, 0, 0,
      0, sigma(posVarianceDiag[2]), 0, 0,
      0, 0, sigma(posVarianceDiag[1]), 0,
      center[0] * SCENE_SCALE, center[1] * SCENE_SCALE, center[2] * SCENE_SCALE, 1,
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
