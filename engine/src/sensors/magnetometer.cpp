#include "sensors/magnetometer.hpp"
#include "constants.hpp"

#include <cmath>

namespace orbitforge::sensors {

// IGRF-13 (2020) Gauss coefficients [nT].  32-byte table; no file I/O at WASM init.
// Source: Alken et al. (2021) Earth Planets Space 73:49
// Order: g10, g11, h11, g20  (math.md §5.3)
const double MagnetometerSensor::k_igrf_coeffs[4] = {
    -29404.5,   // g10 [nT] — axial dipole component
      -1450.7,  // g11 [nT] — equatorial dipole (cos-longitude)
       4652.9,  // h11 [nT] — equatorial dipole (sin-longitude)
      -2499.7   // g20 [nT] — axial quadrupole
};

MagnetometerSensor::MagnetometerSensor(double sigma, unsigned seed)
    : sigma_nT(sigma), rng_(seed), dist_(0.0, sigma)
{}

Eigen::Vector3d MagnetometerSensor::field_eci(const Eigen::Vector3d& r_eci,
                                               double julian_date)
{
    constexpr double deg_to_rad = 1.7453292519943295769e-2;

    // ECI → ECEF: rotate by GAST around z-axis
    const double gast_deg = k_gst_epoch_deg +
        k_gst_rate_deg_per_day * (julian_date - k_j2000_jd);
    const double theta = gast_deg * deg_to_rad;
    const double c = std::cos(theta), s = std::sin(theta);

    // R_ecef_eci = Rz(theta): [c s 0; -s c 0; 0 0 1]
    const Eigen::Vector3d r_ecef(
        c * r_eci.x() + s * r_eci.y(),
       -s * r_eci.x() + c * r_eci.y(),
        r_eci.z()
    );

    const double x   = r_ecef.x(), y = r_ecef.y(), z = r_ecef.z();
    const double r   = r_ecef.norm();
    const double rho = std::sqrt(x*x + y*y);   // cylindrical radius (∝ sin colatitude)
    const double a   = k_re / r;               // R_E/r, dimensionless

    // Guard against pole singularity: |rho| < 1 km
    if (rho < 1e3) {
        // At the poles θ→0 or π; B_λ is undefined; approximate as dipole along z
        const double sign = (z >= 0.0) ? 1.0 : -1.0;
        const double Br   = 2.0 * a*a*a * k_igrf_coeffs[0] * sign;
        // B_eci ≈ -Br·r̂ (pointing inward at north pole, outward at south pole)
        return Eigen::Vector3d(0.0, 0.0, -Br);
    }

    const double cos_theta  = z   / r;
    const double sin_theta  = rho / r;
    const double cos_lambda = x   / rho;
    const double sin_lambda = y   / rho;

    const double g10 = k_igrf_coeffs[0];
    const double g11 = k_igrf_coeffs[1];
    const double h11 = k_igrf_coeffs[2];
    const double g20 = k_igrf_coeffs[3];

    // === n=1 dipole terms ===
    const double a3  = a * a * a;
    const double D1  = g11*cos_lambda + h11*sin_lambda;  // azimuthal dipole amplitude

    // B in geocentric spherical: (B_r, B_theta, B_lambda)
    // B_r = -(∂V/∂r);  B_θ = -(1/r)(∂V/∂θ);  B_λ = -(1/(r sinθ))(∂V/∂λ)
    double Br  = 2.0 * a3 * (g10*cos_theta + D1*sin_theta);
    double Bt  = a3 * (g10*sin_theta - D1*cos_theta);
    double Bl  = a3 * (g11*sin_lambda - h11*cos_lambda);

    // === n=2, m=0 quadrupole term (g20) ===
    const double a4   = a3 * a;
    const double cos2 = cos_theta * cos_theta;
    Br += 3.0 * a4 * g20 * 0.5 * (3.0*cos2 - 1.0);
    Bt += a4 * 3.0 * g20 * sin_theta * cos_theta;
    // B_lambda(n=2,m=0) = 0 (no azimuthal dependence for m=0)

    // === Convert spherical → ECEF Cartesian ===
    // r̂ = (sinθ cosλ, sinθ sinλ, cosθ)
    // θ̂ = (cosθ cosλ, cosθ sinλ, -sinθ)   [points toward geographic south]
    // λ̂ = (-sinλ, cosλ, 0)                [points east]
    const Eigen::Vector3d r_hat(sin_theta*cos_lambda, sin_theta*sin_lambda, cos_theta);
    const Eigen::Vector3d t_hat(cos_theta*cos_lambda, cos_theta*sin_lambda, -sin_theta);
    const Eigen::Vector3d l_hat(-sin_lambda, cos_lambda, 0.0);

    const Eigen::Vector3d B_ecef = Br*r_hat + Bt*t_hat + Bl*l_hat;

    // ECEF → ECI: R_eci_ecef = Rz(-theta) = Rz(theta)^T
    // x_eci = c·x_ecef - s·y_ecef;  y_eci = s·x_ecef + c·y_ecef;  z_eci = z_ecef
    return Eigen::Vector3d(
        c * B_ecef.x() - s * B_ecef.y(),
        s * B_ecef.x() + c * B_ecef.y(),
        B_ecef.z()
    );
}

Eigen::Vector3d MagnetometerSensor::measure(const Eigen::Vector3d& r_eci,
                                             double julian_date)
{
    return field_eci(r_eci, julian_date) +
           Eigen::Vector3d(dist_(rng_), dist_(rng_), dist_(rng_));
}

} // namespace orbitforge::sensors
