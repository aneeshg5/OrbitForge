import type { StateFrame } from './wasm_types.js'

// sizeof(StateFrame) in bytes — must match C++ struct (engine/include/wasm_api.hpp).
// 46 doubles (1 simTime + 3+3 true + (3+3+6+1)×3 filters = 46) × 8 bytes = 368,
// + 1 byte activeFault padded up to the next 8-byte boundary = 376 bytes.
// Verified against the compiler's sizeof(StateFrame): see docs/checkpoint.md.
const FRAME_BYTES = 376

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
  private readonly f64: Float64Array
  private readonly u8: Uint8Array
  private readonly writePosView: Int32Array
  private readonly readPosView: Int32Array
  private readonly capacity: number
  private readPos = 0

  // ringBufferPtr is the byte offset returned by get_ring_buffer_ptr() —
  // the address of the SPSCRingBuffer *within* WASM linear memory, which is
  // not byte 0 (the Simulation singleton, and its ring buffer member, live
  // wherever Emscripten's allocator placed them). The header layout from
  // that offset is fixed by engine/include/memory/ring_buffer.hpp: a
  // 64-byte write_pos_ slot, a 64-byte read_pos_ slot, then frames.
  constructor(sab: SharedArrayBuffer, ringBufferPtr: number, capacity: number) {
    if (ringBufferPtr % 8 !== 0) {
      throw new Error(`ring buffer pointer ${ringBufferPtr} is not 8-byte aligned`)
    }
    this.capacity = capacity
    this.writePosView = new Int32Array(sab, ringBufferPtr, 1)
    this.readPosView  = new Int32Array(sab, ringBufferPtr + 64, 1)
    const framesOffset = ringBufferPtr + 128
    this.f64 = new Float64Array(sab, framesOffset)
    this.u8  = new Uint8Array(sab, framesOffset)
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
