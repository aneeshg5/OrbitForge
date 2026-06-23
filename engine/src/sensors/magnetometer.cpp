#include "sensors/magnetometer.hpp"
#include "constants.hpp"

#include <cmath>

namespace orbitforge::sensors {

// math.md §5.3.
const double MagnetometerSensor::k_igrf_coeffs[4] = {
    -29404.5,
      -1450.7,
       4652.9,
      -2499.7
};

MagnetometerSensor::MagnetometerSensor(double sigma, unsigned seed)
    : sigma_nT(sigma), rng_(seed), dist_(0.0, sigma)
{}

Eigen::Vector3d MagnetometerSensor::field_eci(const Eigen::Vector3d& r_eci,
                                               double julian_date)
{
    constexpr double deg_to_rad = 1.7453292519943295769e-2;

    const double gast_deg = k_gst_epoch_deg +
        k_gst_rate_deg_per_day * (julian_date - k_j2000_jd);
    const double theta = gast_deg * deg_to_rad;
    const double c = std::cos(theta), s = std::sin(theta);

    const Eigen::Vector3d r_ecef(
        c * r_eci.x() + s * r_eci.y(),
       -s * r_eci.x() + c * r_eci.y(),
        r_eci.z()
    );

    const double x   = r_ecef.x(), y = r_ecef.y(), z = r_ecef.z();
    const double r   = r_ecef.norm();
    const double rho = std::sqrt(x*x + y*y);
    const double a   = k_re / r;

    if (rho < 1e3) {
        const double sign = (z >= 0.0) ? 1.0 : -1.0;
        const double Br   = 2.0 * a*a*a * k_igrf_coeffs[0] * sign;
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

    const double a3  = a * a * a;
    const double D1  = g11*cos_lambda + h11*sin_lambda;

    double Br  = 2.0 * a3 * (g10*cos_theta + D1*sin_theta);
    double Bt  = a3 * (g10*sin_theta - D1*cos_theta);
    double Bl  = a3 * (g11*sin_lambda - h11*cos_lambda);

    const double a4   = a3 * a;
    const double cos2 = cos_theta * cos_theta;
    Br += 3.0 * a4 * g20 * 0.5 * (3.0*cos2 - 1.0);
    Bt += a4 * 3.0 * g20 * sin_theta * cos_theta;

    const Eigen::Vector3d r_hat(sin_theta*cos_lambda, sin_theta*sin_lambda, cos_theta);
    const Eigen::Vector3d t_hat(cos_theta*cos_lambda, cos_theta*sin_lambda, -sin_theta);
    const Eigen::Vector3d l_hat(-sin_lambda, cos_lambda, 0.0);

    const Eigen::Vector3d B_ecef = Br*r_hat + Bt*t_hat + Bl*l_hat;

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

Eigen::Vector3d MagnetometerSensor::measure_body(const Eigen::Vector3d& r_eci,
                                                  double julian_date,
                                                  const Eigen::Matrix3d& R_body_eci)
{
    const Eigen::Vector3d b_body = R_body_eci * field_eci(r_eci, julian_date);
    return b_body + Eigen::Vector3d(dist_(rng_), dist_(rng_), dist_(rng_));
}

}
