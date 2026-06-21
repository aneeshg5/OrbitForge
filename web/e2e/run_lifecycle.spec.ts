import { test, expect, type Page } from '@playwright/test'

// Reads the T+ readout's underlying sim-seconds, parsing main.ts's
// formatSimTime() output (HH:MM:SS, or D:HH:MM:SS once a day has passed)
// back into a single number so tests can assert ordering/freshness without
// hardcoding an exact expected value (sim_speed and tool/CI timing jitter
// make exact values flaky; relative comparisons are what actually matter).
async function readSimSeconds(page: Page): Promise<number> {
  const text = (await page.locator('#sim-time-value').textContent()) ?? ''
  const parts = text.split(':').map(Number)
  if (parts.length === 4) {
    const [d, h, m, s] = parts
    return d * 86400 + h * 3600 + m * 60 + s
  }
  const [h, m, s] = parts
  return h * 3600 + m * 60 + s
}

async function waitForTleLoaded(page: Page): Promise<void> {
  // Resolves once the status line settles on a non-loading state — either
  // a live or cached-fallback load (see tle_feed.ts) — rather than a fixed
  // sleep, since CelesTrak's own latency is exactly what varies in
  // practice (this whole feature exists because of that variance).
  await expect(page.locator('#scenario-editor-container .status-line')).not.toHaveClass(/loading/, { timeout: 15_000 })
  await expect(page.locator('#scenario-editor-container .status-line')).toContainText('Loaded', { timeout: 15_000 })
}

test.beforeEach(async ({ page }) => {
  await page.goto('/')
  await waitForTleLoaded(page)
})

test('TLE loads and Run becomes enabled', async ({ page }) => {
  await expect(page.getByRole('button', { name: '▶ Run' })).toBeEnabled()
})

test('Run advances T+ and Pause freezes it', async ({ page }) => {
  await page.getByRole('button', { name: '▶ Run' }).click()
  await expect(page.getByRole('button', { name: '⏸ Pause' })).toBeVisible()

  await page.waitForTimeout(1000)
  const whileRunning = await readSimSeconds(page)
  expect(whileRunning).toBeGreaterThan(0)

  await page.getByRole('button', { name: '⏸ Pause' }).click()
  const atPause = await readSimSeconds(page)
  await page.waitForTimeout(1000)
  const afterPause = await readSimSeconds(page)
  expect(afterPause).toBe(atPause) // frozen, not still ticking
})

test('Reset while running does not crash and clears T+ back to zero', async ({ page }) => {
  await page.getByRole('button', { name: '▶ Run' }).click()
  await page.waitForTimeout(1000)
  expect(await readSimSeconds(page)).toBeGreaterThan(0)

  // The actual regression this guards: resetting mid-run used to hard-crash
  // the tab (ring_reader.ts's readPos never resyncing after
  // reset_simulation() zeroes the producer's counters — see RingReader.drain()).
  await page.getByRole('button', { name: '⟳ Reset' }).click()

  // If the page crashed, this evaluate (and everything after it) throws.
  await page.evaluate(() => document.title)
  await expect(page.getByRole('button', { name: '▶ Run' })).toBeVisible()
  await expect.poll(() => readSimSeconds(page), { timeout: 5000 }).toBe(0)
})

test('Reset/Run cycled rapidly does not crash', async ({ page }) => {
  for (let i = 0; i < 5; i++) {
    await page.getByRole('button', { name: /Run|Continue/ }).click()
    await page.waitForTimeout(300)
    await page.getByRole('button', { name: '⟳ Reset' }).click()
    await page.waitForTimeout(150)
  }
  await page.evaluate(() => document.title) // throws if the tab died
  expect(await readSimSeconds(page)).toBe(0)
})

test('sim speed: default is fast enough to be meaningful, and the typed input clamps', async ({ page }) => {
  const numberInput = page.locator('.sim-speed-number')
  await expect(numberInput).toHaveValue('4320')

  await numberInput.fill('999999')
  await numberInput.blur()
  // SIM_SPEED_MAX in scenario_editor.ts — dt = 0.01 * sim_speed is the
  // RK4 integrator's actual step length (engine/src/wasm_api.cpp), so this
  // isn't a cosmetic UI cap, it's the boundary of the validated regime.
  await expect(numberInput).toHaveValue('5000')
})
