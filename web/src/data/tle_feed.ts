import { parseTle, type OrbitalElements } from './tle_parser.js'

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
// this exact failure mode: a VPN's exit IP getting blocked/rate-limited by
// celestrak.org while every other site works fine). These are genuinely
// real elements fetched live from CelesTrak (not fabricated/approximated —
// see CLAUDE.md's "real satellites, real data" premise), just possibly
// stale by the time they're used as a fallback. cachedAt records exactly
// when each one was captured so the UI can be honest about staleness
// rather than presenting a fallback as if it were live.
interface FallbackEntry {
  name: string
  line1: string
  line2: string
  cachedAt: string // ISO date
}

const FALLBACK_CACHED_AT = '2026-06-20'

const FALLBACK_TLES: Record<number, FallbackEntry> = {
  25544: {
    name: 'ISS (ZARYA)',
    line1: '1 25544U 98067A   26171.41461525  .00008813  00000+0  16600-3 0  9990',
    line2: '2 25544  51.6327 284.1189 0004557 208.5194 151.5545 15.49333088572250',
    cachedAt: FALLBACK_CACHED_AT,
  },
  44714: {
    name: 'STARLINK-1008',
    line1: '1 44714U 19074B   26171.40542176  .00056365  00000+0  97291-3 0  9992',
    line2: '2 44714  53.1521  53.3154 0002110 147.6682 212.4457 15.50487193364717',
    cachedAt: FALLBACK_CACHED_AT,
  },
  26407: {
    name: 'GPS BIIR-5',
    line1: '1 26407U 00040A   26171.26496353  .00000022  00000+0  00000+0 0  9990',
    line2: '2 26407  54.8510 215.0400 0120997 302.4995  43.5641  2.00557702190007',
    cachedAt: FALLBACK_CACHED_AT,
  },
  41866: {
    name: 'GOES 16',
    line1: '1 41866U 16071A   26171.35461541 -.00000095  00000+0  00000+0 0  9999',
    line2: '2 41866   0.3504  85.5238 0000306 339.5740 226.4328  1.00271436 35133',
    cachedAt: FALLBACK_CACHED_AT,
  },
  33791: {
    name: 'COSMOS 2251 DEB',
    line1: '1 33791U 93036AG  26171.15517340  .00001529  00000+0  45236-3 0  9996',
    line2: '2 33791  74.1736  48.4760 0029098 184.3166 335.9513 14.44017382907851',
    cachedAt: FALLBACK_CACHED_AT,
  },
}

export interface TleFetchResult {
  elements: OrbitalElements
  fromCache: boolean
  cachedAt?: string // ISO date the fallback was captured; only set when fromCache is true
}

// The historical /satcat/tle.php?CATNR= endpoint was deprecated in 2020
// and removed in 2022 (confirmed live: it now returns an HTML notice, not
// a TLE) — replaced by the "GP data" API CelesTrak migrated to.
export async function fetchTleByNorad(noradId: number): Promise<TleFetchResult> {
  const url = `https://celestrak.org/NORAD/elements/gp.php?CATNR=${noradId}&FORMAT=TLE`
  try {
    const resp = await fetch(url)
    if (!resp.ok) throw new Error(`CelesTrak fetch failed: ${resp.status}`)
    const text = await resp.text()
    const lines = text.trim().split('\n').map(l => l.trimEnd())
    if (lines.length < 3) throw new Error('Unexpected TLE response from CelesTrak')
    return { elements: parseTle(lines[0], lines[1], lines[2]), fromCache: false }
  } catch (err) {
    const fallback = FALLBACK_TLES[noradId]
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
