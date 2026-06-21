import type { StateFrame } from './wasm_types.js'

// sizeof(StateFrame) in bytes — must match C++ struct (engine/include/wasm_api.hpp).
// Phase 5 grew this substantially (true_quat/omega + both EKF/UKF gaining
// quat+omega+6 more cov_diag entries each). 640 bytes, measured via
// offsetof() against the actual compiled struct — not hand-counted, per
// CLAUDE.md §22 rule 6 ("don't guess at... terms").
const FRAME_BYTES = 640

// Offsets (in doubles, i.e. byte_offset / 8) for each field in StateFrame.
// Must exactly mirror the C++ struct layout — these are the offsetof()
// values measured against the compiled struct, not hand-counted.
const OFF = {
  simTime:     0,
  truePos:     1,
  trueVel:     4,
  trueQuat:    7,
  trueOmega:   11,
  kfPos:       14,
  kfVel:       17,
  kfCovDiag:   20,
  kfNis:       26,
  ekfPos:      27,
  ekfVel:      30,
  ekfQuat:     33,
  ekfOmega:    37,
  ekfCovDiag:  40,
  ekfNis:      52,
  ukfPos:      53,
  ukfVel:      56,
  ukfQuat:     59,
  ukfOmega:    63,
  ukfCovDiag:  66,
  ukfNis:      78,
  activeFault: 79, // uint8, read separately (byte offset = 79*8 = 632)
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

  // Reads n consecutive doubles starting at float64 index `base + off`.
  private readN(base: number, off: number, n: number): number[] {
    const out = new Array<number>(n)
    for (let i = 0; i < n; i++) out[i] = this.f64[base + off + i]!
    return out
  }

  /**
   * Returns pending frames plus whether the producer's counters were just
   * observed to wrap back to (near) zero — the authoritative signal that
   * reset_simulation() actually ran (engine/include/memory/ring_buffer.hpp's
   * clear()), as opposed to main.ts's own optimistic client-side clear the
   * instant Reset is clicked (which can't know exactly when the WASM side
   * catches up, so a frame or two from just before the real reset can
   * still land after that optimistic clear). Callers should treat `reset`
   * as the cue to clear any state derived from frames, even if they
   * already did so optimistically.
   */
  drain(): { frames: StateFrame[]; reset: boolean } {
    const writePos = Atomics.load(this.writePosView, 0)

    // readPos only ever increases, so once it's ahead of a freshly-zeroed
    // writePos, the loop below can never reach readPos === writePos again
    // through normal increments — left unhandled, this spins effectively
    // forever inside a single requestAnimationFrame tick (confirmed: this
    // is what was hard-locking/crashing the tab on Reset). Resync instead
    // of looping.
    if (writePos < this.readPos) {
      this.readPos = writePos
      Atomics.store(this.readPosView, 0, this.readPos)
      return { frames: [], reset: true }
    }

    const frames: StateFrame[] = []

    while (this.readPos !== writePos) {
      const slot = this.readPos & (this.capacity - 1)
      const base = (slot * FRAME_BYTES) / 8  // float64 index

      const frame: StateFrame = {
        simTime:    this.f64[base + OFF.simTime]!,
        truePos:    this.readN(base, OFF.truePos, 3) as [number, number, number],
        trueVel:    this.readN(base, OFF.trueVel, 3) as [number, number, number],
        trueQuat:   this.readN(base, OFF.trueQuat, 4) as [number, number, number, number],
        trueOmega:  this.readN(base, OFF.trueOmega, 3) as [number, number, number],

        kfPos:      this.readN(base, OFF.kfPos, 3) as [number, number, number],
        kfVel:      this.readN(base, OFF.kfVel, 3) as [number, number, number],
        kfCovDiag:  this.readN(base, OFF.kfCovDiag, 6) as [number, number, number, number, number, number],
        kfNis:      this.f64[base + OFF.kfNis]!,

        ekfPos:     this.readN(base, OFF.ekfPos, 3) as [number, number, number],
        ekfVel:     this.readN(base, OFF.ekfVel, 3) as [number, number, number],
        ekfQuat:    this.readN(base, OFF.ekfQuat, 4) as [number, number, number, number],
        ekfOmega:   this.readN(base, OFF.ekfOmega, 3) as [number, number, number],
        ekfCovDiag: this.readN(base, OFF.ekfCovDiag, 12) as StateFrame['ekfCovDiag'],
        ekfNis:     this.f64[base + OFF.ekfNis]!,

        ukfPos:     this.readN(base, OFF.ukfPos, 3) as [number, number, number],
        ukfVel:     this.readN(base, OFF.ukfVel, 3) as [number, number, number],
        ukfQuat:    this.readN(base, OFF.ukfQuat, 4) as [number, number, number, number],
        ukfOmega:   this.readN(base, OFF.ukfOmega, 3) as [number, number, number],
        ukfCovDiag: this.readN(base, OFF.ukfCovDiag, 12) as StateFrame['ukfCovDiag'],
        ukfNis:     this.f64[base + OFF.ukfNis]!,

        activeFault: this.u8[slot * FRAME_BYTES + OFF.activeFault * 8]!,
      }

      frames.push(frame)
      this.readPos++
    }

    Atomics.store(this.readPosView, 0, this.readPos)
    return { frames, reset: false }
  }
}
