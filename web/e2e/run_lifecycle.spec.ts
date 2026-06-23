import { test, expect, type Page } from '@playwright/test'

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

test('Run MC works without ever clicking the topbar Run button first', async ({ page }) => {
  await page.locator('text=Monte Carlo').click()
  const mcButton = page.getByRole('button', { name: '▶ Run MC' })

  await expect(mcButton).toBeEnabled()

  await page.locator('.mc-number-input').first().fill('20')
  await mcButton.click()
  await expect(page.locator('.mc-panel .status-line')).toContainText('Done', { timeout: 10_000 })
  const firstDataRow = await page.locator('.mc-rms-table tbody tr').first().textContent()
  expect(firstDataRow).not.toContain('NaN')
})

test('Run advances T+ and Pause freezes it', async ({ page }) => {
  await page.getByRole('button', { name: '▶ Run' }).click()
  await expect(page.getByRole('button', { name: '⏸ Pause' })).toBeVisible()

  await expect.poll(() => readSimSeconds(page), { timeout: 10_000 }).toBeGreaterThan(0)

  await page.getByRole('button', { name: '⏸ Pause' }).click()
  const atPause = await readSimSeconds(page)
  await page.waitForTimeout(1000)
  const afterPause = await readSimSeconds(page)
  expect(afterPause).toBe(atPause)
})

test('Reset while running does not crash and clears T+ back to zero', async ({ page }) => {
  await page.getByRole('button', { name: '▶ Run' }).click()
  await expect.poll(() => readSimSeconds(page), { timeout: 10_000 }).toBeGreaterThan(0)

  await page.getByRole('button', { name: '⟳ Reset' }).click()

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
  await page.evaluate(() => document.title)
  expect(await readSimSeconds(page)).toBe(0)
})

test('sim speed: default is fast enough to be meaningful, and the typed input clamps', async ({ page }) => {
  const numberInput = page.locator('.sim-speed-number')
  await expect(numberInput).toHaveValue('4320')

  await numberInput.fill('999999')
  await numberInput.blur()
  await expect(numberInput).toHaveValue('5000')
})

test('sim speed: changing it while paused takes effect on Continue', async ({ page }) => {
  const speedInput = page.locator('.sim-speed-number')
  await speedInput.fill('10')
  await speedInput.dispatchEvent('change')

  await page.getByRole('button', { name: '▶ Run' }).click()
  await expect.poll(() => readSimSeconds(page), { timeout: 10_000 }).toBeGreaterThan(0)
  await page.waitForTimeout(800)
  await page.getByRole('button', { name: '⏸ Pause' }).click()
  const slowAdvance = await readSimSeconds(page)

  await speedInput.fill('2000')
  await speedInput.dispatchEvent('change')
  await page.getByRole('button', { name: '▶ Continue' }).click()
  await page.waitForTimeout(800)
  await page.getByRole('button', { name: '⏸ Pause' }).click()
  const fastAdvance = (await readSimSeconds(page)) - slowAdvance

  expect(fastAdvance).toBeGreaterThan(slowAdvance * 10)
})

test('run duration: auto-stops at the target and Continue does not immediately re-trigger it', async ({ page }) => {
  await page.locator('.duration-number').fill('10')
  await page.locator('.duration-unit').selectOption('60')

  await page.getByRole('button', { name: '▶ Run' }).click()
  await expect.poll(() => readSimSeconds(page), { timeout: 10_000 }).toBeGreaterThanOrEqual(600)
  await expect(page.getByRole('button', { name: '▶ Continue' })).toBeVisible()
  const atAutoStop = await readSimSeconds(page)

  await page.getByRole('button', { name: '▶ Continue' }).click()
  await expect(page.getByRole('button', { name: '⏸ Pause' })).toBeVisible()
  await expect.poll(() => readSimSeconds(page), { timeout: 5000 }).toBeGreaterThan(atAutoStop)
})
