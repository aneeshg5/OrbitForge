#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

#include "constants.hpp"

namespace orbitforge::sensors {

// math.md §5.1.
class GpsSensor {
public:
    double sigma_m;

    explicit GpsSensor(double sigma = 10.0, unsigned seed = 42)
        : sigma_m(sigma), rng_(seed), dist_(0.0, sigma)
    {}

    Eigen::Vector3d measure(const Eigen::Vector3d& r_eci, double julian_date) {
        const Eigen::Vector3d r_ecef = R_ecef_eci(julian_date) * r_eci;
        return r_ecef + Eigen::Vector3d(dist_(rng_), dist_(rng_), dist_(rng_));
    }

    static Eigen::Matrix3d R_ecef_eci(double julian_date) {
        const double theta = gast_rad(julian_date);
        const double c = std::cos(theta), s = std::sin(theta);
        Eigen::Matrix3d R;
        R <<  c, s, 0,
             -s, c, 0,
              0, 0, 1;
        return R;
    }

    static double gast_rad(double julian_date) {
        constexpr double deg_to_rad = 1.7453292519943295769e-2;
        const double deg = k_gst_epoch_deg +
            k_gst_rate_deg_per_day * (julian_date - k_j2000_jd);
        return deg * deg_to_rad;
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;
};

}
