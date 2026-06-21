import { defineConfig, devices } from '@playwright/test'

// Bare-minimum e2e coverage for the run/pause/reset lifecycle and TLE
// loading — the gaps that let a hard browser-crashing bug (ring_reader.ts's
// stale readPos after reset_simulation()) and a silent regression (Earth's
// spin/T+ going imperceptible at the default sim speed) both ship
// unnoticed. Needs a real browser (WebGL2 + WASM + SharedArrayBuffer), so
// this is Playwright, not a DOM-only test runner.
export default defineConfig({
  testDir: './e2e',
  timeout: 30_000,
  fullyParallel: false, // all specs share one dev server instance; avoid worker contention on the WASM module
  retries: 0,
  reporter: 'list',
  use: {
    baseURL: 'http://localhost:5173',
    trace: 'retain-on-failure',
  },
  projects: [{ name: 'chromium', use: { ...devices['Desktop Chrome'] } }],
  webServer: {
    command: 'npm run dev',
    url: 'http://localhost:5173',
    reuseExistingServer: true,
    timeout: 30_000,
  },
})
