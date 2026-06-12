#include "dynamics/eom.hpp"
#include "dynamics/perturbations.hpp"
#include "constants.hpp"

#include <cmath>

namespace orbitforge::dynamics {

Eigen::Vector3d compute_gravity(const Eigen::Vector3d& pos_eci) {
    // Two-body: a = -μ/|r|³ · r  (math.md §1.1)
    const double r = pos_eci.norm();
    return (-orbitforge::k_mu / (r * r * r)) * pos_eci;
}

Eigen::Vector3d compute_j2(const Eigen::Vector3d& pos_eci) {
    // J2 oblateness perturbation (math.md §1.2)
    const double x = pos_eci.x();
    const double y = pos_eci.y();
    const double z = pos_eci.z();
    const double r2 = pos_eci.squaredNorm();
    const double r  = std::sqrt(r2);

    const double factor = (1.5) * orbitforge::k_j2
                          * orbitforge::k_mu
                          * (orbitforge::k_re * orbitforge::k_re)
                          / (r2 * r2 * r);

    const double z_r2 = (z / r) * (z / r);

    return Eigen::Vector3d{
        factor * x * (5.0 * z_r2 - 1.0),
        factor * y * (5.0 * z_r2 - 1.0),
        factor * z * (5.0 * z_r2 - 3.0)
    };
}

Eigen::Vector3d compute_drag(
    const Eigen::Vector3d& pos_eci,
    const Eigen::Vector3d& vel_eci,
    double drag_coeff,
    double area_to_mass)
{
    // a_drag = -½ · ρ · C_D · (A/m) · |v_rel|² · v̂_rel  (math.md §1.3)
    const double alt = altitude_from_eci(pos_eci);
    const double rho = atmospheric_density(alt);
    if (rho == 0.0) return Eigen::Vector3d::Zero();

    // Velocity relative to rotating atmosphere: v_rel = v - ω_E × r
    const Eigen::Vector3d omega_earth{0.0, 0.0, orbitforge::k_omega_e};
    const Eigen::Vector3d v_rel = vel_eci - omega_earth.cross(pos_eci);

    const double v_rel_mag = v_rel.norm();
    if (v_rel_mag == 0.0) return Eigen::Vector3d::Zero();

    return -0.5 * rho * drag_coeff * area_to_mass
           * v_rel_mag * v_rel;
}

Eigen::Vector3d compute_srp(
    const Eigen::Vector3d& pos_eci,
    double julian_date,
    double reflectivity_coeff,
    double area_to_mass)
{
    // a_srp = -P_sr · C_R · (A/m) · r̂_sun  (math.md §1.4)
    const Eigen::Vector3d r_sun = sun_direction_eci(julian_date);
    return -orbitforge::k_p_sr * reflectivity_coeff * area_to_mass * r_sun;
}

Eigen::Vector3d compute_acceleration(
    const Eigen::Vector3d& pos_eci,
    const Eigen::Vector3d& vel_eci,
    double julian_date,
    const PerturbationConfig& cfg)
{
    Eigen::Vector3d accel = compute_gravity(pos_eci);

    if (cfg.enable_j2)
        accel += compute_j2(pos_eci);

    if (cfg.enable_drag)
        accel += compute_drag(pos_eci, vel_eci, cfg.drag_coeff, cfg.area_to_mass);

    if (cfg.enable_srp)
        accel += compute_srp(pos_eci, julian_date, 1.3, cfg.area_to_mass);

    return accel;
}

} // namespace orbitforge::dynamics
