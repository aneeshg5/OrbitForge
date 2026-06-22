// Reads the WASM-side Monte Carlo progress counter (a std::atomic<uint32_t>,
// engine/include/monte_carlo/mc_runner.hpp's mc_progress_counter()) directly
// off the shared WASM heap via Atomics — the same "read raw SharedArrayBuffer
// memory independent of whatever the worker's own JS call stack is doing"
// pattern RingReader already uses. Necessary here because run_monte_carlo()'s
// ccall blocks the worker thread for the whole campaign (worker.ts's doc
// comment on runMonteCarlo()): nothing on that thread can run, including
// posting progress messages, until it returns — but the native threads it
// spawns keep incrementing this counter the whole time, and the main
// thread (never blocked by the worker) can poll it independently.
export class McProgressReader {
  private readonly view: Uint32Array

  // ptr is the byte offset returned by get_mc_progress_ptr() — stable for
  // the life of the program (mc_progress_counter() is a Meyers singleton,
  // wasm_api.cpp), so callers fetch it once and reuse it across any number
  // of campaigns rather than re-querying per run.
  constructor(sab: SharedArrayBuffer, ptr: number) {
    if (ptr % 4 !== 0) throw new Error(`mc progress pointer ${ptr} is not 4-byte aligned`)
    this.view = new Uint32Array(sab, ptr, 1)
  }

  read(): number {
    return Atomics.load(this.view, 0)
  }
}
