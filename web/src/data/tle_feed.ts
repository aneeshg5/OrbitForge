import { parseTle, type OrbitalElements } from './tle_parser.js'
import fallbackTlesData from './fallback_tles.json'

export interface SatellitePreset {
  name: string
  noradId: number
  category: string
  whyInteresting: string
}

export const PRESETS: SatellitePreset[] = [
  { name: 'ISS (ZARYA)',       noradId: 25544, category: 'Station',  whyInteresting: '408 km, 51.6° incl., high drag, fastest NEES demo' },
  { name: 'STARLINK-1008',     noradId: 44714, category: 'Starlink', whyInteresting: '550 km, 53°, high A/m, drag dominates' },
  // GPS BIIR-2 (NORAD 26360) stopped publishing GP data (decommissioned) —
  // swapped for BIIR-5, same generation/orbit class, currently active.
  // Same kind of swap as STARLINK-1007 -> -1008 earlier (see checkpoint.md).
  { name: 'GPS BIIR-5',        noradId: 26407, category: 'GPS',      whyInteresting: '20,200 km, medium perturbations, position accuracy demo' },
  { name: 'GOES-16',           noradId: 41866, category: 'Weather',  whyInteresting: '35,786 km GEO, SRP dominates, low filter update rate' },
  { name: 'COSMOS 2251 DEB',   noradId: 33791, category: 'Debris',   whyInteresting: 'High-eccentricity debris — filter divergence demo' },
]

// Last-known-good TLEs for each preset, used when the live CelesTrak fetch
// fails (network block, CelesTrak outage, etc. — see the session notes on
// this exact failure mode: a VPN's/ISP's exit IP getting blocked/rate-limited
// by celestrak.org while every other site works fine). These are genuinely
// real elements fetched live from CelesTrak (not fabricated/approximated —
// see CLAUDE.md's "real satellites, real data" premise), just possibly
// stale by the time they're used as a fallback. cachedAt records exactly
// when each one was captured so the UI can be honest about staleness rather
// than presenting a fallback as if it were live.
//
// Lives in fallback_tles.json (not inline here) so scripts/refresh_fallback_tles.mjs
// can regenerate it as plain data — see .github/workflows/refresh_fallback_tles.yml,
// which runs that script weekly and commits the result.
interface FallbackEntry {
  name: string
  line1: string
  line2: string
  cachedAt: string // ISO date
}

const FALLBACK_TLES: Record<string, FallbackEntry> = fallbackTlesData

export interface TleFetchResult {
  elements: OrbitalElements
  fromCache: boolean
  cachedAt?: string // ISO date the fallback was captured; only set when fromCache is true
}

// A working request measures well under 1s (see checkpoint notes) — 8s is
// generous headroom for real latency while cutting off the case that
// actually hurts: a network silently dropping the connection (firewall/IP
// block, see tle_feed's fallback-table comment above) leaves fetch() to
// hang on the browser's own default timeout, which can run 60s+ with zero
// feedback. AbortController turns that into a fast, predictable failure
// that falls through to the cached fallback below.
const FETCH_TIMEOUT_MS = 8000

// The historical /satcat/tle.php?CATNR= endpoint was deprecated in 2020
// and removed in 2022 (confirmed live: it now returns an HTML notice, not
// a TLE) — replaced by the "GP data" API CelesTrak migrated to.
export async function fetchTleByNorad(noradId: number): Promise<TleFetchResult> {
  const url = `https://celestrak.org/NORAD/elements/gp.php?CATNR=${noradId}&FORMAT=TLE`
  try {
    const resp = await fetch(url, { signal: AbortSignal.timeout(FETCH_TIMEOUT_MS) })
    if (!resp.ok) throw new Error(`CelesTrak fetch failed: ${resp.status}`)
    const text = await resp.text()
    const lines = text.trim().split('\n').map(l => l.trimEnd())
    if (lines.length < 3) throw new Error('Unexpected TLE response from CelesTrak')
    return { elements: parseTle(lines[0], lines[1], lines[2]), fromCache: false }
  } catch (err) {
    const fallback = FALLBACK_TLES[String(noradId)]
    if (!fallback) throw err
    return {
      elements: parseTle(fallback.name, fallback.line1, fallback.line2),
      fromCache: true,
      cachedAt: fallback.cachedAt,
    }
  }
}

export async function fetchAllPresets(): Promise<TleFetchResult[]> {
  return Promise.all(PRESETS.map(p => fetchTleByNorad(p.noradId)))
}
