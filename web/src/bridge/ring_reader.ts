import type { StateFrame } from './wasm_types.js'

const FRAME_BYTES = 640

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
  activeFault: 79,
} as const

export class RingReader {
  private readonly f64: Float64Array
  private readonly u8: Uint8Array
  private readonly writePosView: Int32Array
  private readonly readPosView: Int32Array
  private readonly capacity: number
  private readPos = 0

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

  private readN(base: number, off: number, n: number): number[] {
    const out = new Array<number>(n)
    for (let i = 0; i < n; i++) out[i] = this.f64[base + off + i]!
    return out
  }

  drain(): { frames: StateFrame[]; reset: boolean } {
    const writePos = Atomics.load(this.writePosView, 0)

    // readPos only increases, so once it's ahead of a freshly-zeroed
    // writePos the loop below can never converge through normal increments.
    // Resync instead of spinning.
    if (writePos < this.readPos) {
      this.readPos = writePos
      Atomics.store(this.readPosView, 0, this.readPos)
      return { frames: [], reset: true }
    }

    const frames: StateFrame[] = []

    while (this.readPos !== writePos) {
      const slot = this.readPos & (this.capacity - 1)
      const base = (slot * FRAME_BYTES) / 8

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
