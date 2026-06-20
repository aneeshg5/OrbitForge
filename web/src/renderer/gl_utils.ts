// Minimal WebGL2 + 4x4 matrix helpers shared by earth.ts, orbit.ts, and
// covariance.ts. No external 3D library (no three.js) — raw WebGL2,
// the same approach Figma uses for its rendering engine, not a wrapper
// library.

/** Column-major 4x4 matrix, matching WebGL's convention. */
export type Mat4 = Float32Array

export function mat4Identity(): Mat4 {
  return new Float32Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ])
}

export function mat4Multiply(a: Mat4, b: Mat4): Mat4 {
  const out = new Float32Array(16)
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      let sum = 0
      for (let k = 0; k < 4; k++) {
        sum += a[k * 4 + row]! * b[col * 4 + k]!
      }
      out[col * 4 + row] = sum
    }
  }
  return out
}

/** Rotation about the Z axis (the camera's default depth/forward axis at azimuth=elevation=0). */
export function mat4RotateZ(angleRad: number): Mat4 {
  const c = Math.cos(angleRad)
  const s = Math.sin(angleRad)
  const out = mat4Identity()
  out[0] = c
  out[1] = s
  out[4] = -s
  out[5] = c
  return out
}

/** Rotation about the Y axis (Earth's pole, per earth.ts's sphere parameterization). */
export function mat4RotateY(angleRad: number): Mat4 {
  const c = Math.cos(angleRad)
  const s = Math.sin(angleRad)
  const out = mat4Identity()
  out[0] = c
  out[2] = -s
  out[8] = s
  out[10] = c
  return out
}

export function mat4Perspective(fovYRad: number, aspect: number, near: number, far: number): Mat4 {
  const f = 1.0 / Math.tan(fovYRad / 2)
  const out = new Float32Array(16)
  out[0] = f / aspect
  out[5] = f
  out[10] = (far + near) / (near - far)
  out[11] = -1
  out[14] = (2 * far * near) / (near - far)
  return out
}

/** Right-handed look-at view matrix. */
export function mat4LookAt(eye: [number, number, number], target: [number, number, number], up: [number, number, number]): Mat4 {
  const zx = eye[0] - target[0], zy = eye[1] - target[1], zz = eye[2] - target[2]
  const zLen = Math.hypot(zx, zy, zz) || 1
  const z: [number, number, number] = [zx / zLen, zy / zLen, zz / zLen]

  const xx = up[1] * z[2] - up[2] * z[1]
  const xy = up[2] * z[0] - up[0] * z[2]
  const xz = up[0] * z[1] - up[1] * z[0]
  const xLen = Math.hypot(xx, xy, xz) || 1
  const x: [number, number, number] = [xx / xLen, xy / xLen, xz / xLen]

  const y: [number, number, number] = [
    z[1] * x[2] - z[2] * x[1],
    z[2] * x[0] - z[0] * x[2],
    z[0] * x[1] - z[1] * x[0],
  ]

  const out = new Float32Array(16)
  out[0] = x[0]; out[1] = y[0]; out[2] = z[0]; out[3] = 0
  out[4] = x[1]; out[5] = y[1]; out[6] = z[1]; out[7] = 0
  out[8] = x[2]; out[9] = y[2]; out[10] = z[2]; out[11] = 0
  out[12] = -(x[0] * eye[0] + x[1] * eye[1] + x[2] * eye[2])
  out[13] = -(y[0] * eye[0] + y[1] * eye[1] + y[2] * eye[2])
  out[14] = -(z[0] * eye[0] + z[1] * eye[1] + z[2] * eye[2])
  out[15] = 1
  return out
}

// Zeroes the translation column of a view matrix produced by mat4LookAt(),
// leaving only the rotation. Used for skybox-style "infinitely far"
// backdrops (starfield.ts): with translation stripped, the eye's actual
// position (and therefore zoom distance) has no effect on the projected
// position of a point at a fixed radius — only camera rotation moves it.
// Without this, a starfield placed at a finite radius would visibly drift
// as the camera zooms in/out, which reads as wrong (real stars don't get
// closer when you zoom toward a planet).
export function mat4StripTranslation(view: Mat4): Mat4 {
  const out = new Float32Array(view)
  out[12] = 0
  out[13] = 0
  out[14] = 0
  return out
}

export function compileShader(gl: WebGL2RenderingContext, type: number, source: string): WebGLShader {
  const shader = gl.createShader(type)
  if (!shader) throw new Error('createShader failed')
  gl.shaderSource(shader, source)
  gl.compileShader(shader)
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    const log = gl.getShaderInfoLog(shader)
    gl.deleteShader(shader)
    throw new Error(`shader compile failed: ${log ?? 'unknown error'}`)
  }
  return shader
}

export function createProgram(gl: WebGL2RenderingContext, vsSource: string, fsSource: string): WebGLProgram {
  const vs = compileShader(gl, gl.VERTEX_SHADER, vsSource)
  const fs = compileShader(gl, gl.FRAGMENT_SHADER, fsSource)
  const program = gl.createProgram()
  if (!program) throw new Error('createProgram failed')
  gl.attachShader(program, vs)
  gl.attachShader(program, fs)
  gl.linkProgram(program)
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    const log = gl.getProgramInfoLog(program)
    gl.deleteProgram(program)
    throw new Error(`program link failed: ${log ?? 'unknown error'}`)
  }
  gl.deleteShader(vs)
  gl.deleteShader(fs)
  return program
}

/** Scene-unit scale: 1 scene unit = 1 Earth radius (k_re = 6.3781e6 m). */
export const SCENE_SCALE = 1 / 6.3781e6

/**
 * Remaps an ECI vector (x toward vernal equinox, z toward celestial north
 * — see CLAUDE.md §6) onto scene axes, where scene-Y is the pole (matching
 * earth.ts's sphere and the camera's world-Y-centric orbit convention —
 * see main.ts's EARTH_TILT comment for why the remap lives here instead of
 * on Earth's mesh). (x,y,z) -> (x,z,-y) is a proper -90deg rotation about
 * X (determinant +1), not a raw component swap, so handedness is
 * preserved. Does not apply SCENE_SCALE — callers scale before or after
 * as appropriate (orbit/covariance data is in meters; the solar system's
 * synthetic positions are already in scene units).
 */
export function eciToScene(v: readonly [number, number, number]): [number, number, number] {
  return [v[0], v[2], -v[1]]
}

// Normalized (0-1) RGB matching index.html's --accent-blue/--accent-teal/
// --accent-orange tokens and panels.ts's Chart.js series colors, so the
// same filter reads as the same color in the 3D scene and the charts.
export const FILTER_COLOR_RGB = {
  kf: [0.357, 0.549, 1.0] as const,
  ekf: [0.176, 0.851, 0.769] as const,
  ukf: [0.969, 0.663, 0.243] as const,
}
