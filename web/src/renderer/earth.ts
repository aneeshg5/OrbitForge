// WebGL2 Earth renderer: textured sphere + Fresnel atmosphere rim glow.
// Renders at a fixed radius of 1 scene unit (see gl_utils.SCENE_SCALE —
// orbit.ts and covariance.ts use the same scale so everything composites
// in one scene).

import { createProgram, type Mat4 } from './gl_utils.js'

const VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;

out vec3 v_normalWorld;
out vec3 v_viewDir;
out vec2 v_uv;

void main() {
  vec4 worldPos = u_model * vec4(a_position, 1.0);
  vec4 viewPos = u_view * worldPos;
  gl_Position = u_proj * viewPos;

  v_normalWorld = mat3(u_model) * a_normal;
  // Camera position in world space is the inverse-view translation; since
  // u_view has no scale, the eye position is -transpose(R)*t — simpler to
  // pass view-space position and use its negation as the view direction.
  v_viewDir = -viewPos.xyz;
  v_uv = a_uv;
}
`

const FRAGMENT_SRC = `#version 300 es
precision highp float;

uniform sampler2D u_earthTex;
uniform bool u_hasTexture;
uniform vec3 u_fallbackColor;

in vec3 v_normalWorld;
in vec3 v_viewDir;
in vec2 v_uv;

out vec4 outColor;

void main() {
  vec3 albedo = u_hasTexture ? texture(u_earthTex, v_uv).rgb : u_fallbackColor;

  vec3 n = normalize(v_normalWorld);
  vec3 v = normalize(v_viewDir);
  float fresnel = pow(1.0 - max(dot(n, v), 0.0), 3.0);
  vec3 atmosphere = vec3(0.3, 0.6, 1.0) * fresnel * 0.8;

  outColor = vec4(albedo + atmosphere, 1.0);
}
`

interface SphereGeometry {
  positions: Float32Array
  normals: Float32Array
  uvs: Float32Array
  indices: Uint16Array | Uint32Array
}

// Standard lat/lon UV sphere. Triangle count grows as latBands*lonBands*2.
function buildSphere(radius: number, latBands: number, lonBands: number): SphereGeometry {
  const positions: number[] = []
  const normals: number[] = []
  const uvs: number[] = []
  const indices: number[] = []

  for (let lat = 0; lat <= latBands; lat++) {
    const theta = (lat * Math.PI) / latBands // 0 (north pole) .. pi (south pole)
    const sinTheta = Math.sin(theta)
    const cosTheta = Math.cos(theta)

    for (let lon = 0; lon <= lonBands; lon++) {
      const phi = (lon * 2 * Math.PI) / lonBands
      const sinPhi = Math.sin(phi)
      const cosPhi = Math.cos(phi)

      const x = cosPhi * sinTheta
      const y = cosTheta
      const z = sinPhi * sinTheta

      positions.push(radius * x, radius * y, radius * z)
      normals.push(x, y, z)
      uvs.push(lon / lonBands, lat / latBands)
    }
  }

  for (let lat = 0; lat < latBands; lat++) {
    for (let lon = 0; lon < lonBands; lon++) {
      const first = lat * (lonBands + 1) + lon
      const second = first + lonBands + 1
      indices.push(first, second, first + 1)
      indices.push(second, second + 1, first + 1)
    }
  }

  const IndexArray = positions.length / 3 > 65535 ? Uint32Array : Uint16Array
  return {
    positions: new Float32Array(positions),
    normals: new Float32Array(normals),
    uvs: new Float32Array(uvs),
    indices: new IndexArray(indices),
  }
}

export class EarthRenderer {
  private readonly gl: WebGL2RenderingContext
  private readonly program: WebGLProgram
  private readonly vao: WebGLVertexArrayObject
  private readonly indexCount: number
  private readonly indexType: number
  private readonly texture: WebGLTexture
  private hasTexture = false

  private readonly uModel: WebGLUniformLocation | null
  private readonly uView: WebGLUniformLocation | null
  private readonly uProj: WebGLUniformLocation | null
  private readonly uHasTexture: WebGLUniformLocation | null
  private readonly uFallbackColor: WebGLUniformLocation | null

  constructor(gl: WebGL2RenderingContext, textureUrl = '/earth_8k.jpg') {
    this.gl = gl
    this.program = createProgram(gl, VERTEX_SRC, FRAGMENT_SRC)

    const geom = buildSphere(1.0, 48, 96)
    this.indexCount = geom.indices.length
    this.indexType = geom.indices instanceof Uint32Array ? gl.UNSIGNED_INT : gl.UNSIGNED_SHORT

    const vao = gl.createVertexArray()
    if (!vao) throw new Error('createVertexArray failed')
    this.vao = vao
    gl.bindVertexArray(vao)

    this.bindAttribBuffer(0, geom.positions, 3)
    this.bindAttribBuffer(1, geom.normals, 3)
    this.bindAttribBuffer(2, geom.uvs, 2)

    const indexBuf = gl.createBuffer()
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indexBuf)
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, geom.indices, gl.STATIC_DRAW)
    gl.bindVertexArray(null)

    this.uModel = gl.getUniformLocation(this.program, 'u_model')
    this.uView = gl.getUniformLocation(this.program, 'u_view')
    this.uProj = gl.getUniformLocation(this.program, 'u_proj')
    this.uHasTexture = gl.getUniformLocation(this.program, 'u_hasTexture')
    this.uFallbackColor = gl.getUniformLocation(this.program, 'u_fallbackColor')

    const tex = gl.createTexture()
    if (!tex) throw new Error('createTexture failed')
    this.texture = tex
    this.loadTexture(textureUrl)
  }

  private bindAttribBuffer(location: number, data: Float32Array, size: number): void {
    const gl = this.gl
    const buf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, buf)
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW)
    gl.enableVertexAttribArray(location)
    gl.vertexAttribPointer(location, size, gl.FLOAT, false, 0, 0)
  }

  // Texture asset (web/public/earth_8k.jpg, NASA Blue Marble) is not
  // present in this repo checkout — it must be supplied separately. Falls
  // back to a flat ocean-blue sphere with the same Fresnel atmosphere
  // shading so the renderer still produces a reasonable result with the
  // asset missing, rather than a blank/black sphere or a thrown error.
  private loadTexture(url: string): void {
    const gl = this.gl
    const image = new Image()
    image.onload = () => {
      gl.bindTexture(gl.TEXTURE_2D, this.texture)
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, image)
      gl.generateMipmap(gl.TEXTURE_2D)
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_LINEAR)
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT)
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
      this.hasTexture = true
    }
    image.onerror = () => {
      console.warn(`EarthRenderer: failed to load ${url} — rendering fallback color sphere`)
      this.hasTexture = false
    }
    image.src = url
  }

  render(model: Mat4, view: Mat4, proj: Mat4): void {
    const gl = this.gl
    gl.useProgram(this.program)
    gl.bindVertexArray(this.vao)

    gl.uniformMatrix4fv(this.uModel, false, model)
    gl.uniformMatrix4fv(this.uView, false, view)
    gl.uniformMatrix4fv(this.uProj, false, proj)
    gl.uniform1i(this.uHasTexture, this.hasTexture ? 1 : 0)
    gl.uniform3f(this.uFallbackColor, 0.08, 0.25, 0.45)

    if (this.hasTexture) {
      gl.activeTexture(gl.TEXTURE0)
      gl.bindTexture(gl.TEXTURE_2D, this.texture)
    }

    gl.enable(gl.DEPTH_TEST)
    gl.drawElements(gl.TRIANGLES, this.indexCount, this.indexType, 0)
    gl.bindVertexArray(null)
  }
}
