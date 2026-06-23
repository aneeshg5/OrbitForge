#pragma once

#include <Eigen/Dense>

namespace orbitforge::dynamics {

// See docs/math.md §1.3 for band table.
double atmospheric_density(double altitude_m);

double altitude_from_eci(const Eigen::Vector3d& pos_eci);

// See docs/math.md §1.4.
Eigen::Vector3d sun_direction_eci(double julian_date);

}
