#!/usr/bin/env node
import { readFile, writeFile } from 'node:fs/promises'

// Regenerates web/src/data/fallback_tles.json from live CelesTrak data — the
// automated counterpart to the "manual maintenance" idea: instead of someone
// remembering to update the cached fallback, .github/workflows/refresh_fallback_tles.yml
// runs this weekly and commits the result if anything changed.
//
// NORAD IDs here must stay in sync with web/src/data/tle_feed.ts's PRESETS
// (only 5 entries, changes rarely — not worth a shared-import build step for
// a script that runs standalone in CI, outside the Vite/tsc pipeline).
const NORAD_IDS = [25544, 44714, 26407, 41866, 33791]

const FETCH_TIMEOUT_MS = 8000
const OUTPUT_PATH = new URL('../src/data/fallback_tles.json', import.meta.url)

async function fetchOne(noradId) {
  const url = `https://celestrak.org/NORAD/elements/gp.php?CATNR=${noradId}&FORMAT=TLE`
  const resp = await fetch(url, { signal: AbortSignal.timeout(FETCH_TIMEOUT_MS) })
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`)
  const text = await resp.text()
  const lines = text.trim().split('\n').map((l) => l.trimEnd())
  if (lines.length < 3 || !lines[1].startsWith('1 ') || !lines[2].startsWith('2 ')) {
    throw new Error(`unexpected response: ${text.slice(0, 80)}`)
  }
  return { name: lines[0].trim(), line1: lines[1], line2: lines[2] }
}

async function main() {
  const existing = JSON.parse(await readFile(OUTPUT_PATH, 'utf8'))
  const today = new Date().toISOString().slice(0, 10)
  const next = { ...existing }
  let successCount = 0

  for (const noradId of NORAD_IDS) {
    const key = String(noradId)
    try {
      const { name, line1, line2 } = await fetchOne(noradId)
      next[key] = { name, line1, line2, cachedAt: today }
      successCount++
      console.log(`OK   ${key} ${name}`)
    } catch (err) {
      // Keep whatever was already cached for this satellite rather than
      // blowing away good data because of one bad fetch — a partial outage
      // shouldn't regress the other four presets.
      console.warn(`FAIL ${key}: ${String(err)} — keeping existing cached entry`)
    }
  }

  if (successCount === 0) {
    console.error('All fetches failed — leaving fallback_tles.json untouched.')
    process.exit(1)
  }

  await writeFile(OUTPUT_PATH, JSON.stringify(next, null, 2) + '\n')
  console.log(`Updated ${successCount}/${NORAD_IDS.length} entries.`)
}

main()
