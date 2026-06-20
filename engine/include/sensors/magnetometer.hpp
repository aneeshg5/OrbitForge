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
    // Predates the Phase 5 attitude state — not consumed by any filter;
    // see measure_body() below for the actual H_mag measurement (math.md §7.4).
    Eigen::Vector3d measure(const Eigen::Vector3d& r_eci, double julian_date);

    // Noiseless IGRF-13 dipole field in ECI [nT].
    // Uses g10, g11, h11 (n=1) and g20 (n=2, m=0) coefficients.
    static Eigen::Vector3d field_eci(const Eigen::Vector3d& r_eci, double julian_date);

    // The actual attitude-observable measurement (Phase 5, math.md §7.4):
    // rotates the ECI field into the body frame via R_body_eci (pass
    // q_true.conjugate().toRotationMatrix() for the true trajectory, or
    // q_ref.conjugate().toRotationMatrix() for a filter's predicted
    // measurement — see math/quaternion.hpp's convention) and adds
    // body-frame noise. field_eci()/measure() above stay ECI-frame and
    // are unrelated to this.
    Eigen::Vector3d measure_body(const Eigen::Vector3d& r_eci, double julian_date,
                                  const Eigen::Matrix3d& R_body_eci);

private:
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;

    // IGRF-13 (2020) Gauss coefficients [nT]: g10, g11, h11, g20
    static const double k_igrf_coeffs[4];
};

} // namespace orbitforge::sensors
