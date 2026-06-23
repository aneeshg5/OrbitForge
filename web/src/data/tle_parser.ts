export interface OrbitalElements {
  epochJD: number;
  inclination: number;
  raan: number;
  eccentricity: number;
  argPerigee: number;
  meanAnomaly: number;
  meanMotion: number;
  noradId: number;
  name: string;
  tleLine1: string;
  tleLine2: string;
}

export function parseTle(name: string, line1: string, line2: string): OrbitalElements {
  if (line1.length < 69 || line2.length < 69) {
    throw new Error('TLE lines must be at least 69 characters')
  }
  if (line1[0] !== '1' || line2[0] !== '2') {
    throw new Error('Invalid TLE line designators')
  }

  const epochStr = line1.slice(18, 32).trim()
  const epochJD = tleEpochToJD(epochStr)

  return {
    epochJD,
    inclination:   parseFloat(line2.slice(8, 16)),
    raan:          parseFloat(line2.slice(17, 25)),
    eccentricity:  parseFloat('0.' + line2.slice(26, 33).trim()),
    argPerigee:    parseFloat(line2.slice(34, 42)),
    meanAnomaly:   parseFloat(line2.slice(43, 51)),
    meanMotion:    parseFloat(line2.slice(52, 63)),
    noradId:       parseInt(line1.slice(2, 7), 10),
    name:          name.trim(),
    tleLine1:      line1,
    tleLine2:      line2,
  }
}

function tleEpochToJD(epochStr: string): number {
  const year2d = parseInt(epochStr.slice(0, 2), 10)
  const year = year2d >= 57 ? 1900 + year2d : 2000 + year2d
  const dayOfYear = parseFloat(epochStr.slice(2))

  const yp = year - 1
  const a = Math.floor(yp / 100)
  const b = 2 - a + Math.floor(a / 4)
  const jd0 = Math.floor(365.25 * (yp + 4716)) + 428 + b - 1524.5

  return jd0 + dayOfYear
}
