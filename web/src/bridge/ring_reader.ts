import type { StateFrame } from './wasm_types.js'

// sizeof(StateFrame) in bytes — must match C++ struct (see wasm_api.cpp).
// 8 bytes × (1 + 6 + 6+1 + 6+1 + 6+1) fields + 1 byte activeFault padded to 8 = 312 bytes
const FRAME_BYTES = 312

// Offsets (in doubles, i.e. byte_offset / 8) for each field in StateFrame.
// Must exactly mirror the C++ struct layout.
const OFF = {
  simTime:    0,
  truePos:    1,
  trueVel:    4,
  kfPos:      7,
  kfVel:      10,
  kfCovDiag:  13,
  kfNis:      19,
  ekfPos:     20,
  ekfVel:     23,
  ekfCovDiag: 26,
  ekfNis:     32,
  ukfPos:     33,
  ukfVel:     36,
  ukfCovDiag: 39,
  ukfNis:     45,
  activeFault: 46, // uint8, read separately
} as const

export class RingReader {
  private readonly buf: SharedArrayBuffer
  private readonly f64: Float64Array
  private readonly u8: Uint8Array
  private readonly writePosView: Int32Array
  private readonly readPosView: Int32Array
  private readonly capacity: number
  private readPos = 0

  constructor(sab: SharedArrayBuffer, capacity: number) {
    this.buf = sab
    this.capacity = capacity
    // Head/tail live at the start of the buffer (two 64-byte-aligned Int32 slots)
    this.writePosView = new Int32Array(sab, 0, 1)
    this.readPosView  = new Int32Array(sab, 64, 1)
    // Frames start after 128-byte header (2 × 64-byte cache lines for head/tail)
    this.f64 = new Float64Array(sab, 128)
    this.u8  = new Uint8Array(sab, 128)
  }

  /** Returns all pending frames (may be empty). */
  drain(): StateFrame[] {
    const writePos = Atomics.load(this.writePosView, 0)
    const frames: StateFrame[] = []

    while (this.readPos !== writePos) {
      const slot = this.readPos & (this.capacity - 1)
      const base = (slot * FRAME_BYTES) / 8  // float64 index

      const frame: StateFrame = {
        simTime:    this.f64[base + OFF.simTime],
        truePos:    [this.f64[base + OFF.truePos], this.f64[base + OFF.truePos + 1], this.f64[base + OFF.truePos + 2]],
        trueVel:    [this.f64[base + OFF.trueVel], this.f64[base + OFF.trueVel + 1], this.f64[base + OFF.trueVel + 2]],
        kfPos:      [this.f64[base + OFF.kfPos],   this.f64[base + OFF.kfPos + 1],   this.f64[base + OFF.kfPos + 2]],
        kfVel:      [this.f64[base + OFF.kfVel],   this.f64[base + OFF.kfVel + 1],   this.f64[base + OFF.kfVel + 2]],
        kfCovDiag:  [this.f64[base+OFF.kfCovDiag], this.f64[base+OFF.kfCovDiag+1], this.f64[base+OFF.kfCovDiag+2], this.f64[base+OFF.kfCovDiag+3], this.f64[base+OFF.kfCovDiag+4], this.f64[base+OFF.kfCovDiag+5]],
        kfNis:      this.f64[base + OFF.kfNis],
        ekfPos:     [this.f64[base + OFF.ekfPos],  this.f64[base + OFF.ekfPos + 1],  this.f64[base + OFF.ekfPos + 2]],
        ekfVel:     [this.f64[base + OFF.ekfVel],  this.f64[base + OFF.ekfVel + 1],  this.f64[base + OFF.ekfVel + 2]],
        ekfCovDiag: [this.f64[base+OFF.ekfCovDiag],this.f64[base+OFF.ekfCovDiag+1],this.f64[base+OFF.ekfCovDiag+2],this.f64[base+OFF.ekfCovDiag+3],this.f64[base+OFF.ekfCovDiag+4],this.f64[base+OFF.ekfCovDiag+5]],
        ekfNis:     this.f64[base + OFF.ekfNis],
        ukfPos:     [this.f64[base + OFF.ukfPos],  this.f64[base + OFF.ukfPos + 1],  this.f64[base + OFF.ukfPos + 2]],
        ukfVel:     [this.f64[base + OFF.ukfVel],  this.f64[base + OFF.ukfVel + 1],  this.f64[base + OFF.ukfVel + 2]],
        ukfCovDiag: [this.f64[base+OFF.ukfCovDiag],this.f64[base+OFF.ukfCovDiag+1],this.f64[base+OFF.ukfCovDiag+2],this.f64[base+OFF.ukfCovDiag+3],this.f64[base+OFF.ukfCovDiag+4],this.f64[base+OFF.ukfCovDiag+5]],
        ukfNis:     this.f64[base + OFF.ukfNis],
        activeFault: this.u8[slot * FRAME_BYTES + OFF.activeFault * 8],
      }

      frames.push(frame)
      this.readPos++
    }

    Atomics.store(this.readPosView, 0, this.readPos)
    return frames
  }
}
