import { parseTle, type OrbitalElements } from './tle_parser.js'

export interface SatellitePreset {
  name: string
  noradId: number
  category: string
  whyInteresting: string
}

export const PRESETS: SatellitePreset[] = [
  { name: 'ISS (ZARYA)',       noradId: 25544, category: 'Station',  whyInteresting: '408 km, 51.6° incl., high drag, fastest NEES demo' },
  { name: 'STARLINK-1007',     noradId: 44713, category: 'Starlink', whyInteresting: '550 km, 53°, high A/m, drag dominates' },
  { name: 'GPS BIIR-2',        noradId: 26360, category: 'GPS',      whyInteresting: '20,200 km, medium perturbations, position accuracy demo' },
  { name: 'GOES-16',           noradId: 41866, category: 'Weather',  whyInteresting: '35,786 km GEO, SRP dominates, low filter update rate' },
  { name: 'COSMOS 2251 DEB',   noradId: 33791, category: 'Debris',   whyInteresting: 'High-eccentricity debris — filter divergence demo' },
]

export async function fetchTleByNorad(noradId: number): Promise<OrbitalElements> {
  const url = `https://celestrak.org/satcat/tle.php?CATNR=${noradId}`
  const resp = await fetch(url)
  if (!resp.ok) throw new Error(`CelesTrak fetch failed: ${resp.status}`)
  const text = await resp.text()
  const lines = text.trim().split('\n').map(l => l.trimEnd())
  if (lines.length < 3) throw new Error('Unexpected TLE response from CelesTrak')
  return parseTle(lines[0], lines[1], lines[2])
}

export async function fetchAllPresets(): Promise<OrbitalElements[]> {
  return Promise.all(PRESETS.map(p => fetchTleByNorad(p.noradId)))
}
