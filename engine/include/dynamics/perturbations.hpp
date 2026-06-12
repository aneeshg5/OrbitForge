#pragma once

#include <Eigen/Dense>

namespace orbitforge::dynamics {

// Atmospheric density via 7-band exponential model.
// Returns 0 for altitude above ~1000 km.
// See docs/math.md §1.3 for band table.
double atmospheric_density(double altitude_m);

// Compute altitude from ECI position vector.
double altitude_from_eci(const Eigen::Vector3d& pos_eci);

// Simplified analytical solar position in ECI (unit vector).
// Accurate to ~0.01°. See docs/math.md §1.4.
Eigen::Vector3d sun_direction_eci(double julian_date);

} // namespace orbitforge::dynamics
