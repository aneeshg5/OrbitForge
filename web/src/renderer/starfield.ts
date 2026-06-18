// Procedural starfield backdrop. Not a literal solar-system renderer
// (OrbitForge is an Earth-orbit estimation tool, not a planetary
// visualizer) and not a real star catalog (no RA/Dec data sourced here,
// so constellations aren't astronomically accurate) — this is a believable deep-space
// backdrop: a uniform scattering of dim background stars, a denser/
// brighter band standing in for the Milky Way, and a handful of brighter
// foreground stars, all generated from a fixed seed so the sky is the
// same every load rather than reshuffling on every page refresh.
//
// Rendered as GL_POINTS on a sphere far outside the camera's max zoom
// distance, using a rotation-only view matrix (gl_utils.mat4StripTranslation)
// so the stars behave as though at infinity — camera rotation moves them,
// zooming does not, matching how real stars behave relative to a nearby
// planet.

import { createProgram, type Mat4 } from './gl_utils.js'

const VERTEX_SRC = `#version 300 es
precision highp float;

uniform mat4 u_viewRotation;
uniform mat4 u_proj;

layout(location = 0) in vec3 a_position;
layout(location = 1) in float a_size;
layout(location = 2) in float a_brightness;

out float v_brightness;

void main() {
  gl_Position = u_proj * u_viewRotation * vec4(a_position, 1.0);
  gl_PointSize = a_size;
  v_brightness = a_brightness;
}
`

const FRAGMENT_SRC = `#version 300 es
precision highp float;

in float v_brightness;
out vec4 outColor;

void main() {
  // Soft circular falloff so points render as dots, not squares.
  vec2 centered = gl_PointCoord - vec2(0.5);
  float dist = length(centered);
  if (dist > 0.5) discard;
  float alpha = smoothstep(0.5, 0.0, dist) * v_brightness;
  outColor = vec4(vec3(1.0), alpha);
}
`

const STAR_RADIUS = 50; // scene units — well outside the camera's max distance (40)

// Deterministic PRNG (mulberry32) so the sky is stable across reloads
// instead of reshuffling every time.
function makeRng(seed: number): () => number {
  let a = seed;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Uniform random point on a unit sphere (not uniform-on-each-axis, which
// would visibly cluster stars at the poles).
function randomUnitSphere(rng: () => number): [number, number, number] {
  const u = rng();
  const v = rng();
  const theta = 2 * Math.PI * u;
  const phi = Math.acos(2 * v - 1);
  const sinPhi = Math.sin(phi);
  return [sinPhi * Math.cos(theta), Math.cos(phi), sinPhi * Math.sin(theta)];
}

interface StarBuffers {
  positions: Float32Array;
  sizes: Float32Array;
  brightness: Float32Array;
}

function generateStars(): StarBuffers {
  const rng = makeRng(0x5eed1e55);

  const positions: number[] = [];
  const sizes: number[] = [];
  const brightness: number[] = [];

  // Background stars: most are dim and small — real sky star brightness
  // follows a steep distribution (few bright stars, many faint ones).
  const backgroundCount = 3500;
  for (let i = 0; i < backgroundCount; i++) {
    const [x, y, z] = randomUnitSphere(rng);
    positions.push(x * STAR_RADIUS, y * STAR_RADIUS, z * STAR_RADIUS);
    const mag = Math.pow(rng(), 3); // skew toward dim
    sizes.push(1.0 + mag * 1.5);
    brightness.push(0.15 + mag * 0.55);
  }

  // Milky Way band: extra stars concentrated near a tilted great circle,
  // density falling off with angular distance from the band plane.
  // Tilt is arbitrary (no real galactic-plane data used) — chosen purely
  // so the band reads as a diagonal sweep across the sky rather than
  // lining up suspiciously with any coordinate axis.
  const bandNormal = normalize([0.35, 0.85, -0.4]);
  const bandCount = 6000;
  let bandAdded = 0;
  let guard = 0;
  while (bandAdded < bandCount && guard < bandCount * 20) {
    guard++;
    const [x, y, z] = randomUnitSphere(rng);
    const distFromBand = Math.abs(x * bandNormal[0] + y * bandNormal[1] + z * bandNormal[2]);
    const bandWidth = 0.22;
    if (distFromBand > bandWidth) continue; // rejection sample to concentrate near the band
    const falloff = 1.0 - distFromBand / bandWidth;
    positions.push(x * STAR_RADIUS, y * STAR_RADIUS, z * STAR_RADIUS);
    const mag = Math.pow(rng(), 2);
    sizes.push(1.5 + mag * 1.5);
    // Higher brightness floor than the background population (not lower)
    // — density alone from rejection sampling wasn't reading as a visible
    // band at normal point sizes; boosting per-point brightness too makes
    // the haze actually legible instead of just statistically present.
    brightness.push((0.35 + mag * 0.45) * falloff);
    bandAdded++;
  }

  // A small number of distinctly brighter foreground stars for visual
  // interest — not tied to any real bright-star catalog.
  const brightCount = 40;
  for (let i = 0; i < brightCount; i++) {
    const [x, y, z] = randomUnitSphere(rng);
    positions.push(x * STAR_RADIUS, y * STAR_RADIUS, z * STAR_RADIUS);
    sizes.push(2.2 + rng() * 1.8);
    brightness.push(0.75 + rng() * 0.25);
  }

  return {
    positions: new Float32Array(positions),
    sizes: new Float32Array(sizes),
    brightness: new Float32Array(brightness),
  };
}

function normalize(v: [number, number, number]): [number, number, number] {
  const len = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / len, v[1] / len, v[2] / len];
}

export class Starfield {
  private readonly gl: WebGL2RenderingContext;
  private readonly program: WebGLProgram;
  private readonly vao: WebGLVertexArrayObject;
  private readonly starCount: number;
  private readonly uViewRotation: WebGLUniformLocation | null;
  private readonly uProj: WebGLUniformLocation | null;

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl;
    this.program = createProgram(gl, VERTEX_SRC, FRAGMENT_SRC);

    const stars = generateStars();
    this.starCount = stars.sizes.length;

    const vao = gl.createVertexArray();
    if (!vao) throw new Error('createVertexArray failed');
    this.vao = vao;
    gl.bindVertexArray(vao);

    this.bindAttribBuffer(0, stars.positions, 3);
    this.bindAttribBuffer(1, stars.sizes, 1);
    this.bindAttribBuffer(2, stars.brightness, 1);

    gl.bindVertexArray(null);

    this.uViewRotation = gl.getUniformLocation(this.program, 'u_viewRotation');
    this.uProj = gl.getUniformLocation(this.program, 'u_proj');
  }

  private bindAttribBuffer(location: number, data: Float32Array, size: number): void {
    const gl = this.gl;
    const buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(location);
    gl.vertexAttribPointer(location, size, gl.FLOAT, false, 0, 0);
  }

  /** viewRotation should be gl_utils.mat4StripTranslation(view), not the raw view matrix. */
  render(viewRotation: Mat4, proj: Mat4): void {
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.bindVertexArray(this.vao);

    gl.uniformMatrix4fv(this.uViewRotation, false, viewRotation);
    gl.uniformMatrix4fv(this.uProj, false, proj);

    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.drawArrays(gl.POINTS, 0, this.starCount);
    gl.disable(gl.BLEND);

    gl.bindVertexArray(null);
  }
}
