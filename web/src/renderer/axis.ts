import { createProgram, type Mat4 } from './gl_utils.js'

const VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;

void main() {
  gl_Position = u_proj * u_view * vec4(a_position, 1.0);
}
`

const FRAGMENT_SRC = `#version 300 es
precision highp float;

out vec4 outColor;

void main() {
  outColor = vec4(0.75, 0.8, 0.9, 1.0);
}
`

const SPHERE_RADIUS = 1.0
const AXIS_OVERHANG = 0.4
const DASH_LENGTH = 0.06
const GAP_LENGTH = 0.04

function buildDashedAxisPositions(): Float32Array {
  const half = SPHERE_RADIUS + AXIS_OVERHANG
  const positions: number[] = []
  const step = DASH_LENGTH + GAP_LENGTH
  for (let y = -half; y < half; y += step) {
    const dashEnd = Math.min(y + DASH_LENGTH, half)
    positions.push(0, y, 0, 0, dashEnd, 0)
  }
  return new Float32Array(positions)
}

export class RotationAxisRenderer {
  private readonly gl: WebGL2RenderingContext
  private readonly program: WebGLProgram
  private readonly vao: WebGLVertexArrayObject
  private readonly vertexCount: number
  private readonly uView: WebGLUniformLocation | null
  private readonly uProj: WebGLUniformLocation | null

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl
    this.program = createProgram(gl, VERTEX_SRC, FRAGMENT_SRC)

    const positions = buildDashedAxisPositions()
    this.vertexCount = positions.length / 3

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

    this.uView = gl.getUniformLocation(this.program, 'u_view')
    this.uProj = gl.getUniformLocation(this.program, 'u_proj')
  }

  render(view: Mat4, proj: Mat4): void {
    const gl = this.gl
    gl.useProgram(this.program)
    gl.bindVertexArray(this.vao)
    gl.uniformMatrix4fv(this.uView, false, view)
    gl.uniformMatrix4fv(this.uProj, false, proj)
    gl.drawArrays(gl.LINES, 0, this.vertexCount)
    gl.bindVertexArray(null)
  }
}
