// Polled directly off the shared WASM heap via Atomics: run_monte_carlo()'s
// ccall blocks the worker thread for the whole campaign, so the native
// threads it spawns keep incrementing this counter while the main thread
// (never blocked) polls it independently.
export class McProgressReader {
  private readonly view: Uint32Array

  constructor(sab: SharedArrayBuffer, ptr: number) {
    if (ptr % 4 !== 0) throw new Error(`mc progress pointer ${ptr} is not 4-byte aligned`)
    this.view = new Uint32Array(sab, ptr, 1)
  }

  read(): number {
    return Atomics.load(this.view, 0)
  }
}
