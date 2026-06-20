// Shared direction helper for solar_system.ts's stylized Sun/Moon orbit
// ring — both are placed on a common ecliptic-tilted ring using a fixed
// J2000 obliquity, parameterized by an arbitrary angle rather than any
// real-time solar longitude (the Sun's position here is a fast, watchable
// stylization driven by wall-clock time — see solar_system.ts).

/** Fixed J2000 mean obliquity. */
export const k_obliquity_rad = (23.439 * Math.PI) / 180.0

/**
 * Unit vector in ECI for a point at ecliptic longitude thetaRad, latitude
 * 0, using the fixed J2000 obliquity. Used to place the Sun and Moon in
 * solar_system.ts on a shared ecliptic-tilted ring, ignoring each body's
 * small real orbital inclination to the ecliptic.
 */
export function eclipticDirection(thetaRad: number): [number, number, number] {
  return [Math.cos(thetaRad), Math.cos(k_obliquity_rad) * Math.sin(thetaRad), Math.sin(k_obliquity_rad) * Math.sin(thetaRad)]
}
