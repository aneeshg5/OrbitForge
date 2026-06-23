#pragma once

#include <Eigen/Dense>
#include <random>

namespace orbitforge::sensors {

// math.md §5.3.
class MagnetometerSensor {
public:
    double sigma_nT;

    explicit MagnetometerSensor(double sigma = 100.0, unsigned seed = 42);

    Eigen::Vector3d measure(const Eigen::Vector3d& r_eci, double julian_date);

    static Eigen::Vector3d field_eci(const Eigen::Vector3d& r_eci, double julian_date);

    // math.md §7.4.
    Eigen::Vector3d measure_body(const Eigen::Vector3d& r_eci, double julian_date,
                                  const Eigen::Matrix3d& R_body_eci);

private:
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;

    static const double k_igrf_coeffs[4];
};

}
