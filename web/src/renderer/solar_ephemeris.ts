export const k_obliquity_rad = (23.439 * Math.PI) / 180.0

export function eclipticDirection(thetaRad: number): [number, number, number] {
  return [Math.cos(thetaRad), Math.cos(k_obliquity_rad) * Math.sin(thetaRad), Math.sin(k_obliquity_rad) * Math.sin(thetaRad)]
}
