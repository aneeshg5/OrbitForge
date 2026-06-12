#pragma once

#include <Eigen/Dense>

namespace orbitforge::dynamics {

struct PerturbationConfig {
    bool enable_j2   = true;
    bool enable_drag = true;
    bool enable_srp  = false;
    double drag_coeff   = 2.2;
    double area_to_mass = 0.01;   // m²/kg
};

// Returns total acceleration [ax, ay, az] in ECI (m/s²).
// See docs/math.md §1 for equation references.
Eigen::Vector3d compute_acceleration(
    const Eigen::Vector3d& pos_eci,   // meters
    const Eigen::Vector3d& vel_eci,   // m/s
    double julian_date,
    const PerturbationConfig& cfg
);

// Two-body gravitational acceleration only (no perturbations).
Eigen::Vector3d compute_gravity(const Eigen::Vector3d& pos_eci);

// J2 oblateness perturbation acceleration.
Eigen::Vector3d compute_j2(const Eigen::Vector3d& pos_eci);

// Atmospheric drag acceleration.
// Returns zero vector above model ceiling (~1000 km).
Eigen::Vector3d compute_drag(
    const Eigen::Vector3d& pos_eci,
    const Eigen::Vector3d& vel_eci,
    double drag_coeff,
    double area_to_mass
);

// Solar radiation pressure acceleration.
Eigen::Vector3d compute_srp(
    const Eigen::Vector3d& pos_eci,
    double julian_date,
    double reflectivity_coeff,
    double area_to_mass
);

} // namespace orbitforge::dynamics
