#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace orbitforge::sensors {

// Gyro model: direct body-frame angular-rate measurement with bias random
// walk. z_gyro = omega_true + bias + N(0, sigma_gyro²·I)
// bias(t+dt) = bias(t) + N(0, sigma_bias²·dt·I)
// Bias lives only on this sensor's measured side — deliberately NOT an
// estimated filter state (CLAUDE.md §6.3); the 12-state EKF/UKF treat
// z_gyro as a direct, slightly-noisy measurement of their own omega state
// and absorb the unmodeled bias into R/Q. Mirrors ImuSensor's pattern
// exactly rather than introducing a new one. Defaults: sigma_gyro =
// 0.001 rad/s, sigma_bias = 0.0005 rad/s/√s. See math.md §7.4.
class GyroSensor {
public:
    double          sigma_gyro_rad_s;  // white noise std dev [rad/s]
    double          sigma_bias_walk;   // bias spectral density [rad/s/√s]
    Eigen::Vector3d bias;              // current bias state [rad/s]

    GyroSensor(double sigma_gyro = 0.001,
               double sigma_bias = 0.0005,
               unsigned seed = 43)
        : sigma_gyro_rad_s(sigma_gyro),
          sigma_bias_walk(sigma_bias),
          bias(Eigen::Vector3d::Zero()),
          rng_(seed),
          gyro_dist_(0.0, sigma_gyro),
          unit_dist_(0.0, 1.0)
    {}

    // Advances bias by a random-walk step of size sigma_bias·sqrt(dt),
    // then returns omega_true + bias + N(0, sigma_gyro²·I).
    Eigen::Vector3d measure(const Eigen::Vector3d& omega_true, double dt) {
        if (dt > 0.0) {
            const double sb = sigma_bias_walk * std::sqrt(dt);
            bias += Eigen::Vector3d(
                sb * unit_dist_(rng_),
                sb * unit_dist_(rng_),
                sb * unit_dist_(rng_)
            );
        }
        return omega_true + bias +
               Eigen::Vector3d(gyro_dist_(rng_), gyro_dist_(rng_), gyro_dist_(rng_));
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> gyro_dist_;  // N(0, sigma_gyro²)
    std::normal_distribution<double> unit_dist_;  // N(0,1), scaled by sigma_bias·√dt
};

} // namespace orbitforge::sensors
