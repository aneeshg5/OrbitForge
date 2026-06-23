import { createProgram, eciToScene, type Mat4, mat4Multiply, mat4RotateY, mat4RotateZ } from './gl_utils.js'
import { buildSphere } from './earth.js'
import { eclipticDirection } from './solar_ephemeris.js'

const BODY_VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

out vec3 v_normalWorld;

void main() {
  gl_Position = u_proj * u_view * u_model * vec4(a_position, 1.0);
  v_normalWorld = mat3(u_model) * a_normal;
}
`

const BODY_FRAGMENT_SRC = `#version 300 es
precision highp float;

uniform vec3 u_color;
uniform vec3 u_sunDirWorld;

in vec3 v_normalWorld;
out vec4 outColor;

void main() {
  vec3 n = normalize(v_normalWorld);
  float diffuse = max(dot(n, normalize(u_sunDirWorld)), 0.0);
  float lit = 0.08 + 0.92 * diffuse;
  outColor = vec4(u_color * lit, 1.0);
}
`

const SUN_SPHERE_VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

out vec3 v_normalWorld;
out vec3 v_normalView;
out vec3 v_viewDir;

void main() {
  vec4 viewPos = u_view * u_model * vec4(a_position, 1.0);
  gl_Position = u_proj * viewPos;
  v_normalWorld = mat3(u_model) * a_normal;
  v_normalView = mat3(u_view) * v_normalWorld;
  v_viewDir = -viewPos.xyz;
}
`

const SUN_SPHERE_FRAGMENT_SRC = `#version 300 es
precision highp float;

uniform float u_time;

in vec3 v_normalWorld;
in vec3 v_normalView;
in vec3 v_viewDir;
out vec4 outColor;

float hash31(vec3 p) {
  p = fract(p * vec3(443.897, 441.423, 437.195));
  p += dot(p, p.yzx + 19.19);
  return fract((p.x + p.y) * p.z);
}

float noise3(vec3 p) {
  vec3 i = floor(p);
  vec3 f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  float n000 = hash31(i + vec3(0.0, 0.0, 0.0));
  float n100 = hash31(i + vec3(1.0, 0.0, 0.0));
  float n010 = hash31(i + vec3(0.0, 1.0, 0.0));
  float n110 = hash31(i + vec3(1.0, 1.0, 0.0));
  float n001 = hash31(i + vec3(0.0, 0.0, 1.0));
  float n101 = hash31(i + vec3(1.0, 0.0, 1.0));
  float n011 = hash31(i + vec3(0.0, 1.0, 1.0));
  float n111 = hash31(i + vec3(1.0, 1.0, 1.0));
  float nx00 = mix(n000, n100, f.x);
  float nx10 = mix(n010, n110, f.x);
  float nx01 = mix(n001, n101, f.x);
  float nx11 = mix(n011, n111, f.x);
  float nxy0 = mix(nx00, nx10, f.y);
  float nxy1 = mix(nx01, nx11, f.y);
  return mix(nxy0, nxy1, f.z);
}

float fbm3(vec3 p) {
  float total = 0.0;
  float amp = 0.5;
  for (int i = 0; i < 5; i++) {
    total += noise3(p) * amp;
    p *= 2.02;
    amp *= 0.5;
  }
  return total;
}

void main() {
  vec3 n = normalize(v_normalWorld);
  vec3 nView = normalize(v_normalView);
  vec3 v = normalize(v_viewDir);
  float centerness = max(dot(nView, v), 0.0);

  float t = u_time * 0.05;
  vec3 p = n * 4.0 + vec3(t, -t * 0.6, t * 0.35);
  float turb = fbm3(p) * 0.45 + fbm3(p * 5.5 + 7.0) * 0.35 + fbm3(p * 13.0 + 17.0) * 0.2;
  turb = pow(clamp(turb, 0.0, 1.0), 1.8);

  vec3 black = vec3(0.05, 0.0, 0.0);
  vec3 deepRed = vec3(0.55, 0.06, 0.0);
  vec3 orange = vec3(1.0, 0.35, 0.02);
  vec3 yellowWhite = vec3(1.0, 0.85, 0.45);
  vec3 color = mix(black, deepRed, smoothstep(0.0, 0.35, turb));
  color = mix(color, orange, smoothstep(0.3, 0.65, turb));
  color = mix(color, yellowWhite, smoothstep(0.6, 0.95, turb));

  float flareNoise = fbm3(n * 22.0 - vec3(t * 2.0, t, -t));
  float flare = smoothstep(0.85, 0.92, flareNoise);
  color = mix(color, vec3(1.0, 0.97, 0.85), flare * 0.95);

  float rim = pow(1.0 - centerness, 4.0);
  color += vec3(1.0, 0.4, 0.15) * rim * 0.5;

  outColor = vec4(color, 1.0);
}
`

const GLOW_VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;
uniform float u_pointSize;

layout(location = 0) in vec3 a_position;

void main() {
  gl_Position = u_proj * u_view * u_model * vec4(a_position, 1.0);
  gl_PointSize = u_pointSize;
}
`

const GLOW_FRAGMENT_SRC = `#version 300 es
precision highp float;

uniform float u_alphaScale;
uniform float u_time;

out vec4 outColor;

void main() {
  vec2 centered = gl_PointCoord - vec2(0.5);
  float d = length(centered) * 2.0;
  float angle = atan(centered.y, centered.x);

  float wisp = sin(angle * 5.0 + u_time * 0.3) * 0.5
    + sin(angle * 11.0 - u_time * 0.5 + 1.7) * 0.3
    + sin(angle * 23.0 + u_time * 0.8 + 4.1) * 0.2;
  float radialBoost = 1.0 + wisp * 0.18;

  float falloff = pow(clamp(1.0 - d * radialBoost, 0.0, 1.0), 2.2);
  vec3 color = mix(vec3(1.0, 0.55, 0.15), vec3(1.0, 0.95, 0.78), falloff);
  outColor = vec4(color, falloff * u_alphaScale);
}
`

const SUN_ORBIT_RADIUS = 23481.66
const SUN_ORBIT_PERIOD_SEC = 365.25 * 86400
const SUN_PHASE0 = 1.4
const SUN_MESH_RADIUS = 109.27
const SUN_GLOW_INNER_PX = 70.0
const SUN_GLOW_INNER_ALPHA = 0.4
const SUN_GLOW_OUTER_PX = 620.0
const SUN_GLOW_OUTER_ALPHA = 0.28

const MOON_ORBIT_RADIUS = 60.35
const MOON_MESH_RADIUS = 0.27
const MOON_ORBIT_PERIOD_SEC = 27.321661 * 86400
const MOON_SPIN_PERIOD_SEC = 27.321661 * 86400
const MOON_TILT_DEG = 6.7
const MOON_COLOR: readonly [number, number, number] = [0.65, 0.65, 0.63]
const MOON_PHASE0 = 5.5

function bodyModelMatrix(scenePos: readonly [number, number, number], tiltRad: number, spinRad: number, meshRadius: number): Mat4 {
  const r = mat4Multiply(mat4RotateZ(tiltRad), mat4RotateY(spinRad))
  return new Float32Array([
    r[0]! * meshRadius, r[1]! * meshRadius, r[2]! * meshRadius, 0,
    r[4]! * meshRadius, r[5]! * meshRadius, r[6]! * meshRadius, 0,
    r[8]! * meshRadius, r[9]! * meshRadius, r[10]! * meshRadius, 0,
    scenePos[0], scenePos[1], scenePos[2], 1,
  ])
}

function orbitScenePos(center: readonly [number, number, number], thetaRad: number, radius: number): [number, number, number] {
  const dir = eciToScene(eclipticDirection(thetaRad))
  return [center[0] + dir[0] * radius, center[1] + dir[1] * radius, center[2] + dir[2] * radius]
}

export function sunDirectionScene(simTimeSec: number): [number, number, number] {
  const theta = SUN_PHASE0 + (2 * Math.PI * simTimeSec) / SUN_ORBIT_PERIOD_SEC
  return eciToScene(eclipticDirection(theta))
}

export class SolarSystemRenderer {
  private readonly gl: WebGL2RenderingContext
  private readonly bodyProgram: WebGLProgram
  private readonly sunSphereProgram: WebGLProgram
  private readonly glowProgram: WebGLProgram

  private readonly sphereVao: WebGLVertexArrayObject
  private readonly sphereIndexCount: number
  private readonly sphereIndexType: number

  private readonly glowVao: WebGLVertexArrayObject

  private readonly bUModel: WebGLUniformLocation | null
  private readonly bUView: WebGLUniformLocation | null
  private readonly bUProj: WebGLUniformLocation | null
  private readonly bUColor: WebGLUniformLocation | null
  private readonly bUSunDir: WebGLUniformLocation | null

  private readonly ssUModel: WebGLUniformLocation | null
  private readonly ssUView: WebGLUniformLocation | null
  private readonly ssUProj: WebGLUniformLocation | null
  private readonly ssUTime: WebGLUniformLocation | null

  private readonly gUModel: WebGLUniformLocation | null
  private readonly gUView: WebGLUniformLocation | null
  private readonly gUProj: WebGLUniformLocation | null
  private readonly gUPointSize: WebGLUniformLocation | null
  private readonly gUAlphaScale: WebGLUniformLocation | null
  private readonly gUTime: WebGLUniformLocation | null

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl
    this.bodyProgram = createProgram(gl, BODY_VERTEX_SRC, BODY_FRAGMENT_SRC)
    this.sunSphereProgram = createProgram(gl, SUN_SPHERE_VERTEX_SRC, SUN_SPHERE_FRAGMENT_SRC)
    this.glowProgram = createProgram(gl, GLOW_VERTEX_SRC, GLOW_FRAGMENT_SRC)

    const sphere = buildSphere(1.0, 16, 24)
    this.sphereIndexCount = sphere.indices.length
    this.sphereIndexType = sphere.indices instanceof Uint32Array ? gl.UNSIGNED_INT : gl.UNSIGNED_SHORT
    const sphereVao = gl.createVertexArray()
    if (!sphereVao) throw new Error('createVertexArray failed')
    this.sphereVao = sphereVao
    gl.bindVertexArray(sphereVao)
    this.bindAttribBuffer(0, sphere.positions, 3)
    this.bindAttribBuffer(1, sphere.normals, 3)
    const sphereIndexBuf = gl.createBuffer()
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, sphereIndexBuf)
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, sphere.indices, gl.STATIC_DRAW)
    gl.bindVertexArray(null)

    const glowVao = gl.createVertexArray()
    if (!glowVao) throw new Error('createVertexArray failed')
    this.glowVao = glowVao
    gl.bindVertexArray(glowVao)
    this.bindAttribBuffer(0, new Float32Array([0, 0, 0]), 3)
    gl.bindVertexArray(null)

    this.bUModel = gl.getUniformLocation(this.bodyProgram, 'u_model')
    this.bUView = gl.getUniformLocation(this.bodyProgram, 'u_view')
    this.bUProj = gl.getUniformLocation(this.bodyProgram, 'u_proj')
    this.bUColor = gl.getUniformLocation(this.bodyProgram, 'u_color')
    this.bUSunDir = gl.getUniformLocation(this.bodyProgram, 'u_sunDirWorld')

    this.ssUModel = gl.getUniformLocation(this.sunSphereProgram, 'u_model')
    this.ssUView = gl.getUniformLocation(this.sunSphereProgram, 'u_view')
    this.ssUProj = gl.getUniformLocation(this.sunSphereProgram, 'u_proj')
    this.ssUTime = gl.getUniformLocation(this.sunSphereProgram, 'u_time')

    this.gUModel = gl.getUniformLocation(this.glowProgram, 'u_model')
    this.gUView = gl.getUniformLocation(this.glowProgram, 'u_view')
    this.gUProj = gl.getUniformLocation(this.glowProgram, 'u_proj')
    this.gUPointSize = gl.getUniformLocation(this.glowProgram, 'u_pointSize')
    this.gUAlphaScale = gl.getUniformLocation(this.glowProgram, 'u_alphaScale')
    this.gUTime = gl.getUniformLocation(this.glowProgram, 'u_time')
  }

  private bindAttribBuffer(location: number, data: Float32Array, size: number): void {
    const gl = this.gl
    const buf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, buf)
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW)
    gl.enableVertexAttribArray(location)
    gl.vertexAttribPointer(location, size, gl.FLOAT, false, 0, 0)
  }

  render(view: Mat4, proj: Mat4, sunDirScene: readonly [number, number, number], tSecAnim: number, simTimeSec: number): void {
    const gl = this.gl
    gl.enable(gl.DEPTH_TEST)

    const sunScenePos: [number, number, number] = [
      sunDirScene[0] * SUN_ORBIT_RADIUS,
      sunDirScene[1] * SUN_ORBIT_RADIUS,
      sunDirScene[2] * SUN_ORBIT_RADIUS,
    ]

    gl.useProgram(this.sunSphereProgram)
    gl.bindVertexArray(this.sphereVao)
    const sunModel = bodyModelMatrix(sunScenePos, 0, 0, SUN_MESH_RADIUS)
    gl.uniformMatrix4fv(this.ssUModel, false, sunModel)
    gl.uniformMatrix4fv(this.ssUView, false, view)
    gl.uniformMatrix4fv(this.ssUProj, false, proj)
    gl.uniform1f(this.ssUTime, tSecAnim)
    gl.drawElements(gl.TRIANGLES, this.sphereIndexCount, this.sphereIndexType, 0)
    gl.bindVertexArray(null)

    gl.useProgram(this.bodyProgram)
    gl.uniformMatrix4fv(this.bUView, false, view)
    gl.uniformMatrix4fv(this.bUProj, false, proj)
    gl.uniform3f(this.bUSunDir, sunDirScene[0], sunDirScene[1], sunDirScene[2])

    const moonTheta = MOON_PHASE0 + (2 * Math.PI * simTimeSec) / MOON_ORBIT_PERIOD_SEC
    const moonPos = orbitScenePos([0, 0, 0], moonTheta, MOON_ORBIT_RADIUS)
    const moonSpin = (2 * Math.PI * simTimeSec) / MOON_SPIN_PERIOD_SEC
    const moonTilt = (MOON_TILT_DEG * Math.PI) / 180
    const moonModel = bodyModelMatrix(moonPos, moonTilt, moonSpin, MOON_MESH_RADIUS)
    gl.bindVertexArray(this.sphereVao)
    gl.uniformMatrix4fv(this.bUModel, false, moonModel)
    gl.uniform3f(this.bUColor, MOON_COLOR[0], MOON_COLOR[1], MOON_COLOR[2])
    gl.drawElements(gl.TRIANGLES, this.sphereIndexCount, this.sphereIndexType, 0)
    gl.bindVertexArray(null)

    gl.useProgram(this.glowProgram)
    gl.bindVertexArray(this.glowVao)
    const glowModel = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, sunScenePos[0], sunScenePos[1], sunScenePos[2], 1])
    gl.uniformMatrix4fv(this.gUModel, false, glowModel)
    gl.uniformMatrix4fv(this.gUView, false, view)
    gl.uniformMatrix4fv(this.gUProj, false, proj)
    gl.uniform1f(this.gUTime, tSecAnim)
    gl.enable(gl.BLEND)
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE)
    gl.disable(gl.DEPTH_TEST)
    gl.depthMask(false)
    gl.uniform1f(this.gUPointSize, SUN_GLOW_OUTER_PX)
    gl.uniform1f(this.gUAlphaScale, SUN_GLOW_OUTER_ALPHA)
    gl.drawArrays(gl.POINTS, 0, 1)
    gl.uniform1f(this.gUPointSize, SUN_GLOW_INNER_PX)
    gl.uniform1f(this.gUAlphaScale, SUN_GLOW_INNER_ALPHA)
    gl.drawArrays(gl.POINTS, 0, 1)
    gl.depthMask(true)
    gl.enable(gl.DEPTH_TEST)
    gl.disable(gl.BLEND)
    gl.bindVertexArray(null)
  }
}
