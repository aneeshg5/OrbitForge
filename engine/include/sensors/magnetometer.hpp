#pragma once

#include <Eigen/Dense>
#include <random>

namespace orbitforge::sensors {

// Magnetometer model using IGRF-13 (2020) truncated dipole approximation.
// Coefficients g10, g11, h11, g20 stored as 32-byte const array in .cpp.
// Field computed in geocentric spherical coords from ECI input, returned in ECI. See math.md §5.3.
class MagnetometerSensor {
public:
    double sigma_nT;  // white noise std dev [nT], default 100 nT

    explicit MagnetometerSensor(double sigma = 100.0, unsigned seed = 42);

    // Returns IGRF dipole field in ECI [nT] plus N(0, sigma_nT²·I).
    Eigen::Vector3d measure(const Eigen::Vector3d& r_eci, double julian_date);

    // Noiseless IGRF-13 dipole field in ECI [nT].
    // Uses g10, g11, h11 (n=1) and g20 (n=2, m=0) coefficients.
    static Eigen::Vector3d field_eci(const Eigen::Vector3d& r_eci, double julian_date);

private:
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;

    // IGRF-13 (2020) Gauss coefficients [nT]: g10, g11, h11, g20
    static const double k_igrf_coeffs[4];
};

} // namespace orbitforge::sensors
