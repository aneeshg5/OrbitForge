#include "dynamics/perturbations.hpp"
#include "constants.hpp"

#include <cmath>

namespace orbitforge::dynamics {

// 7-band exponential atmosphere model.
// Columns: h_base (m), h_scale (m), rho_base (kg/m³)
struct AtmoBand {
    double h_base;
    double h_scale;
    double rho_base;
};

static constexpr AtmoBand k_atmo_bands[7] = {
    {       0.0,  8500.0, 1.225   },
    {  100000.0,  5877.0, 5.297e-7},
    {  200000.0,  6667.0, 2.418e-10},
    {  300000.0,  7554.0, 1.916e-11},
    {  400000.0,  8382.0, 2.803e-12},
    {  600000.0,  7714.0, 1.137e-13},
    {  800000.0, 16129.0, 5.245e-14},
};
static constexpr double k_atmo_ceiling = 1.0e6; // 1000 km

double atmospheric_density(double altitude_m) {
    if (altitude_m >= k_atmo_ceiling) return 0.0;
    if (altitude_m < 0.0) altitude_m = 0.0;

    int band = 0;
    for (int i = 6; i >= 0; --i) {
        if (altitude_m >= k_atmo_bands[i].h_base) {
            band = i;
            break;
        }
    }
    const auto& b = k_atmo_bands[band];
    return b.rho_base * std::exp(-(altitude_m - b.h_base) / b.h_scale);
}

double altitude_from_eci(const Eigen::Vector3d& pos_eci) {
    return pos_eci.norm() - orbitforge::k_re;
}

Eigen::Vector3d sun_direction_eci(double julian_date) {
    // Simplified solar longitude (degrees), accurate to ~0.01°.
    // See Vallado "Fundamentals of Astrodynamics" §5.1.
    const double T = (julian_date - orbitforge::k_j2000_jd) / 36525.0;
    const double lambda_deg = 280.460 + 36000.771 * T;
    const double M_deg      = 357.528 + 35999.050 * T;
    const double M_rad      = M_deg * M_PI / 180.0;
    const double lambda_ec  = (lambda_deg + 1.915 * std::sin(M_rad)
                               + 0.020 * std::sin(2.0 * M_rad)) * M_PI / 180.0;
    const double eps_rad    = (23.439 - 0.0130042 * T) * M_PI / 180.0;

    return Eigen::Vector3d{
        std::cos(lambda_ec),
        std::cos(eps_rad) * std::sin(lambda_ec),
        std::sin(eps_rad) * std::sin(lambda_ec)
    };
}

} // namespace orbitforge::dynamics
