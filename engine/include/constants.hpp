#pragma once

namespace orbitforge {

// Gravitational parameter [m³/s²]
constexpr double k_mu       = 3.986004418e14;
// J2 oblateness coefficient [-]
constexpr double k_j2       = 1.08262668e-3;
// Earth mean equatorial radius [m]
constexpr double k_re       = 6.3781e6;
// Earth rotation rate [rad/s]
constexpr double k_omega_e  = 7.2921150e-5;
// Solar radiation pressure at 1 AU [N/m²]
constexpr double k_p_sr     = 4.56e-6;
// Speed of light [m/s]
constexpr double k_c        = 2.99792458e8;
// Astronomical unit [m]
constexpr double k_au       = 1.495978707e11;
// Seconds per day
constexpr double k_sec_per_day = 86400.0;
// J2000 epoch as Julian date
constexpr double k_j2000_jd = 2451545.0;
// Greenwich sidereal time coefficients (degrees)
constexpr double k_gst_epoch_deg   = 280.46061837;
constexpr double k_gst_rate_deg_per_day = 360.98564736629;

} // namespace orbitforge
