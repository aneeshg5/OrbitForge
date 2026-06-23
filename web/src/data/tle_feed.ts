import { parseTle, type OrbitalElements } from './tle_parser.js'
import fallbackTlesData from './fallback_tles.json'

export interface SatellitePreset {
  name: string
  noradId: number
  category: string
  whyInteresting: string
}

export const PRESETS: SatellitePreset[] = [
  { name: 'ISS (ZARYA)',       noradId: 25544, category: 'Station',  whyInteresting: '408 km altitude, 51.6° inclination, high drag, fastest consistency demo' },
  { name: 'STARLINK-1008',     noradId: 44714, category: 'Starlink', whyInteresting: '550 km altitude, 53° inclination, high area to mass, drag dominated' },
  { name: 'GPS BIIR-5',        noradId: 26407, category: 'GPS',      whyInteresting: '20,200 km altitude, medium perturbations, position accuracy demo' },
  { name: 'GOES-16',           noradId: 41866, category: 'Weather',  whyInteresting: '35,786 km altitude (geostationary), sunlight pressure dominates, slow updates' },
  { name: 'COSMOS 2251 DEB',   noradId: 33791, category: 'Debris',   whyInteresting: 'High eccentricity debris orbit, shows filter divergence' },
]

interface FallbackEntry {
  name: string
  line1: string
  line2: string
  cachedAt: string
}

const FALLBACK_TLES: Record<string, FallbackEntry> = fallbackTlesData

export interface TleFetchResult {
  elements: OrbitalElements
  fromCache: boolean
  cachedAt?: string
}

const FETCH_TIMEOUT_MS = 8000

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
