// The Sun and Moon, both orbiting Earth (the scene's fixed origin and
// the camera's fixed target — see main.ts's OrbitCamera).
//
// Earth orbiting the Sun and the Sun "orbiting" Earth are kinematically
// indistinguishable to an Earth-fixed observer: from Earth, the Sun
// appears to sweep around you once per year either way. Since the
// camera here is permanently centered on Earth (a deliberate, explicit
// choice — see CLAUDE.md), the Sun is animated as the thing that moves,
// once per SUN_ORBIT_PERIOD_SEC, compressed from a real year down to a
// watchable loop — the same kind of stylization already used for
// Earth's own spin and the Moon's orbit. This is what makes "Earth
// revolving around the Sun" something you can actually watch: rotate
// the camera and the Sun visibly sweeps around Earth, dragging the
// day/night terminator with it (sunDirectionScene() below feeds Earth's
// lighting directly, so the two are always in sync).
//
// An earlier version rendered the Sun (plus all 7 other planets) on an
// infinitely-distant, zoom-independent "sky backdrop" — mirroring
// starfield.ts — so camera movement never moved them. That read as "the
// Sun is tiny and far away, disconnected from Earth," which is exactly
// backwards from the goal. The Sun is now a real, near scene object,
// like the Moon: rendered with the ordinary (non-stripped) view matrix,
// so it gets real parallax and occlusion against Earth, and is sized
// and distanced to clearly read as the dominant body in the scene.

import { createProgram, eciToScene, type Mat4, mat4Multiply, mat4RotateY, mat4RotateZ } from './gl_utils.js'
import { buildSphere } from './earth.js'
import { eclipticDirection } from './solar_ephemeris.js'

// Shared by the Moon (lit by the Sun, so needs a diffuse term).
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
  // Lower ambient floor than Earth's (no atmosphere to scatter light onto
  // the night side) — reads as an airless body.
  float lit = 0.08 + 0.92 * diffuse;
  outColor = vec4(u_color * lit, 1.0);
}
`

// The Sun's mesh: unlit/emissive by definition (it IS the light source,
// so a diffuse-lighting term makes no sense here) — a simple view-facing
// gradient (bright near-white center fading to warm orange at the limb)
// instead, echoing the glow halo's color below for a consistent look.
const SUN_SPHERE_VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

out vec3 v_normalWorld;
// Same normal, transformed into view space — needed alongside v_viewDir
// (already view-space, see below) for any dot(normal, viewDir) "facing
// the camera" calculation. v_normalWorld stays in world space and feeds
// the turbulence/color pattern instead, which must stay fixed to the
// Sun's own surface rather than rotate with the camera.
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

// Cheap hash-based 3D value noise + fbm — gives the surface a mottled,
// turbulent "plasma" look instead of a single smooth gradient (which is
// what made the old version read as a flat-shaded plastic ball). Sampled
// directly on the sphere's own normal rather than a 2D UV, so there's no
// seam at the poles or the +-180 deg wrap.
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
  // centerness ("how directly this fragment faces the camera") must compare
  // the normal and view direction in the SAME space — both view space here
  // (v_viewDir already is). Comparing a world-space normal against a
  // view-space direction (the previous version) isn't a meaningful
  // geometric quantity; it happened to look plausible as a soft pow()
  // gradient for the old rim-glow-only use, but produced a sharp, non-
  // circular cut once it gated a hard edge-alpha cutoff below.
  vec3 nView = normalize(v_normalView);
  vec3 v = normalize(v_viewDir);
  float centerness = max(dot(nView, v), 0.0);

  // Real photos of the Sun (e.g. SDO imagery) show a mottled, high-contrast
  // surface with dark red/near-black gaps between bright granulation cells
  // and occasional white-hot flares — brightness comes from the plasma
  // itself, not from viewing angle. The previous version's color was driven
  // mostly by a smooth dot(n, viewDir) gradient with noise only as a minor
  // multiplier, which is exactly what made it read as a smoothly shaded
  // ball instead of fire. Here color comes almost entirely from layered
  // turbulence instead, slowly "boiling" via u_time.
  float t = u_time * 0.05;
  vec3 p = n * 4.0 + vec3(t, -t * 0.6, t * 0.35);
  // Three layered frequencies instead of two: coarse cloud-scale shape,
  // a mid cellular layer, and a fine fibrous layer on top — two layers
  // alone still read as smooth soft-edged blobs (too coarse vs. real
  // granulation, which is dense and fibrous all the way down). The third,
  // highest-frequency layer is what breaks up those blobs into the finer
  // mottled texture real Sun photos show.
  float turb = fbm3(p) * 0.45 + fbm3(p * 5.5 + 7.0) * 0.35 + fbm3(p * 13.0 + 17.0) * 0.2;
  turb = pow(clamp(turb, 0.0, 1.0), 1.8); // sharpen contrast: more dark gaps, fewer bright peaks

  vec3 black = vec3(0.05, 0.0, 0.0);
  vec3 deepRed = vec3(0.55, 0.06, 0.0);
  vec3 orange = vec3(1.0, 0.35, 0.02);
  vec3 yellowWhite = vec3(1.0, 0.85, 0.45);
  vec3 color = mix(black, deepRed, smoothstep(0.0, 0.35, turb));
  color = mix(color, orange, smoothstep(0.3, 0.65, turb));
  color = mix(color, yellowWhite, smoothstep(0.6, 0.95, turb));

  // Rare, brighter "flare" hotspots from a separate higher-frequency noise
  // sample thresholded hard — small white-hot specks scattered across the
  // disk, echoing the bright active regions in real Sun imagery. Sampled
  // at a much higher frequency than before (9.0 -> 16.0) with a tighter
  // threshold band, so this reads as several small, sharp, scattered
  // points rather than one or two large soft blobs — a single dominant
  // bright patch was making the whole disk look like a glowing orb with
  // a central hotspot instead of a mottled surface with localized flares.
  float flareNoise = fbm3(n * 22.0 - vec3(t * 2.0, t, -t));
  float flare = smoothstep(0.85, 0.92, flareNoise);
  color = mix(color, vec3(1.0, 0.97, 0.85), flare * 0.95);

  // Thin true-limb brightening only right at the grazing edge (much
  // narrower than before) — hands off smoothly into the additive glow
  // sprite layered on top in render(), without washing out the whole disk.
  float rim = pow(1.0 - centerness, 4.0);
  color += vec3(1.0, 0.4, 0.15) * rim * 0.5;

  // Sphere stays fully opaque — the "not solid" look comes entirely from
  // the additive glow sprites layered outside the mesh in render(), not
  // from fading the mesh itself. An edge-alpha fade was tried here but
  // made the mesh itself look dissolved/broken rather than just adding a
  // glow around a solid sphere, which is what was actually wanted.
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

// Bright near-white core fading to a warm orange halo, additively
// blended — a stylized bloom layered on top of the Sun's mesh, not a
// physically modeled corona. Drawn twice per frame (see render()): a
// smaller/brighter layer right at the mesh edge and a larger/fainter
// layer for the soft bleed-into-space look, both sharing this shader via
// u_alphaScale. The falloff is a continuous pow() curve all the way to
// the sprite's edge (alpha -> 0) rather than the old two-piece smoothstep,
// which left a visible faint disk boundary — that hard edge against black
// space was a big part of why this read as a solid ball instead of a glow.
const GLOW_FRAGMENT_SRC = `#version 300 es
precision highp float;

uniform float u_alphaScale;
uniform float u_time;

out vec4 outColor;

void main() {
  vec2 centered = gl_PointCoord - vec2(0.5);
  float d = length(centered) * 2.0;
  float angle = atan(centered.y, centered.x);

  // Wispy, irregular edge instead of a perfectly smooth radial falloff:
  // a few overlapping angular sine harmonics, slowly animated, perturb how
  // far the glow reaches at each angle — reads as uneven plasma streamers/
  // prominences poking past the limb, the way real corona photos look,
  // rather than a clean circular halo.
  float wisp = sin(angle * 5.0 + u_time * 0.3) * 0.5
    + sin(angle * 11.0 - u_time * 0.5 + 1.7) * 0.3
    + sin(angle * 23.0 + u_time * 0.8 + 4.1) * 0.2;
  float radialBoost = 1.0 + wisp * 0.18;

  float falloff = pow(clamp(1.0 - d * radialBoost, 0.0, 1.0), 2.2);
  vec3 color = mix(vec3(1.0, 0.55, 0.15), vec3(1.0, 0.95, 0.78), falloff);
  outColor = vec4(color, falloff * u_alphaScale);
}
`

// Distance from Earth (origin) to the Sun, in scene units — not to real
// scale (real distance is ~23,500 Earth radii; nothing would fit in the
// same frame as Earth at that distance with a camera that orbits within
// 40 Earth radii — see main.ts's OrbitCamera.k_max_distance). Chosen to
// sit clearly beyond the Moon's orbit (14) while staying comfortably
// inside the camera's max zoom (40), so dragging/zooming the camera
// produces real, visible parallax against the Sun instead of it sitting
// glued to the sky.
const SUN_ORBIT_RADIUS = 22.0
// Real Julian year (365.25 days) — driven by simulated elapsed time, not
// wall-clock (see sunDirectionScene's param and main.ts's call site), so
// Sun/Moon position is physically consistent with the sim's own clock
// instead of an arbitrary cosmetic loop. Earlier versions used a fast
// 90s/30s wall-clock loop so the orbit was watchable in real time, but
// that had no relation to the simulated time elapsed and read as
// inconsistent once the UI started showing elapsed sim time too — at
// typical sim durations (seconds to hours), the Sun's actual annual
// motion is correctly almost imperceptible, same as in reality.
const SUN_ORBIT_PERIOD_SEC = 365.25 * 86400
// Arbitrary starting angle, offset from the Moon's (see MOON_PHASE0)
// just to avoid the two starting collinear.
const SUN_PHASE0 = 1.4
// Real Sun:Earth radius ratio is ~109:1 — not reproduced literally (it
// would dwarf everything else at this scene's scale), but large enough
// to unambiguously read as the dominant body, clearly bigger than Earth
// (radius 1.0, see earth.ts).
const SUN_MESH_RADIUS = 2.2
// Two layered additive sprites for the corona: a tight, bright layer
// sized just past the mesh's on-screen edge (blends the mesh's rim glow
// into the halo with no visible seam) and a much larger, faint layer for
// the soft bleed-into-space look that makes it read as a glowing star
// rather than a solid ball with a hard silhouette.
// Inner layer shrunk and dimmed from its previous size (180px/0.9 alpha):
// at the mesh's typical on-screen size, that layer was large and bright
// enough to nearly cover the whole disk, so the additive bloom washed out
// the mesh's own granulation/flare texture and the Sun read as a single
// smooth glowing orb instead of a mottled surface with a halo around it.
const SUN_GLOW_INNER_PX = 70.0
const SUN_GLOW_INNER_ALPHA = 0.4
const SUN_GLOW_OUTER_PX = 620.0
const SUN_GLOW_OUTER_ALPHA = 0.28

// Moon orbits Earth directly. Real distance is 60.3 Earth radii —
// compressed to fit comfortably inside the camera's max zoom (40)
// without crowding GPS/GEO altitude satellite orbits (~4-6 scene units)
// or the Sun's orbit above. Real orbital/rotation period (27.3d) is
// tidally locked 1:1; not modeled as literal tidal lock here since a
// flat gray sphere has no surface marking to show it.
const MOON_ORBIT_RADIUS = 14.0
const MOON_MESH_RADIUS = 0.27
// Real sidereal period (27.321661 days), driven by simulated elapsed time —
// see SUN_ORBIT_PERIOD_SEC's comment above for why. Spin matches orbit
// period (tidal lock) for the same reason it always did; both are now real
// rather than stylized.
const MOON_ORBIT_PERIOD_SEC = 27.321661 * 86400
const MOON_SPIN_PERIOD_SEC = 27.321661 * 86400
const MOON_TILT_DEG = 6.7
const MOON_COLOR: readonly [number, number, number] = [0.65, 0.65, 0.63]
const MOON_PHASE0 = 5.5

/** Combines axial tilt + spin (rotation) into a model matrix scaled by meshRadius and translated to scenePos. */
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

/**
 * Current Sun direction (Earth -> Sun, unit vector, scene axes), driven by
 * SIMULATED elapsed time (StateFrame.simTime, seconds — 0 if no sim has
 * run yet), not wall-clock — see SUN_ORBIT_PERIOD_SEC's comment. Feeds
 * both this module's own Sun position and Earth's day/night terminator
 * (see main.ts) — keeping the visible Sun and the lit hemisphere always
 * in sync as it orbits.
 */
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

    // Shared unit sphere for the Sun + Moon (each scaled by its own mesh
    // radius via the model matrix) — low-poly since they're small/flat
    // on screen, unlike Earth's textured 48x96 mesh.
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

  /**
   * Renders the Sun (mesh + glow halo) and the Moon, both as real scene
   * objects orbiting Earth — ordinary (non-stripped) view matrix, real
   * depth test, so they correctly parallax/occlude against Earth and
   * each other regardless of draw order. sunDirScene should be
   * sunDirectionScene(simTimeSec) — passed in rather than recomputed so
   * the caller can feed the identical direction to Earth's lighting.
   *
   * Two separate time inputs, deliberately not the same value: tSecAnim
   * is wall-clock seconds, driving only the cosmetic shader animations
   * (the Sun surface's "boiling" turbulence, the corona's wispy edge) —
   * these have no real-world rate to be physically tied to, so they keep
   * animating continuously even while the sim is paused, same as Earth's
   * cosmetic spin (main.ts). simTimeSec is the sim's own elapsed time,
   * driving Sun/Moon ORBITAL position — see SUN_ORBIT_PERIOD_SEC's comment
   * for why that one must be simulated time, not wall-clock.
   */
  render(view: Mat4, proj: Mat4, sunDirScene: readonly [number, number, number], tSecAnim: number, simTimeSec: number): void {
    const gl = this.gl
    gl.enable(gl.DEPTH_TEST)

    const sunScenePos: [number, number, number] = [
      sunDirScene[0] * SUN_ORBIT_RADIUS,
      sunDirScene[1] * SUN_ORBIT_RADIUS,
      sunDirScene[2] * SUN_ORBIT_RADIUS,
    ]

    // Sun mesh: unlit/emissive, turbulent view-facing gradient (see shader
    // comment). Fully opaque, same as Earth/Moon — no blending needed.
    gl.useProgram(this.sunSphereProgram)
    gl.bindVertexArray(this.sphereVao)
    const sunModel = bodyModelMatrix(sunScenePos, 0, 0, SUN_MESH_RADIUS)
    gl.uniformMatrix4fv(this.ssUModel, false, sunModel)
    gl.uniformMatrix4fv(this.ssUView, false, view)
    gl.uniformMatrix4fv(this.ssUProj, false, proj)
    gl.uniform1f(this.ssUTime, tSecAnim)
    gl.drawElements(gl.TRIANGLES, this.sphereIndexCount, this.sphereIndexType, 0)
    gl.bindVertexArray(null)

    // Moon: lit by the same sunDirScene, so its terminator stays in sync
    // with the Sun's visible position and with Earth's.
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

    // Sun glow halo: additively blended, drawn last so it layers on top
    // of the opaque Sun/Moon/Earth. Depth test must be off for this part —
    // the point sprite's depth is the sphere's *center*, which sits behind
    // the mesh's own front-facing surface, so with depth testing on this
    // sprite always lost to the opaque mesh drawn moments earlier and
    // never actually appeared. That silent self-occlusion was why the Sun
    // previously had no halo at all and just looked like a flat-shaded
    // ball — the glow code ran every frame but its output never made it
    // past the depth test.
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
    // Outer layer first (larger, fainter — the soft bleed into space),
    // inner layer on top (smaller, brighter — blends into the mesh's rim).
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
