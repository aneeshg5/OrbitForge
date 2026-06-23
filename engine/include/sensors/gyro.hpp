#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace orbitforge::sensors {

// math.md §7.4.
class GyroSensor {
public:
    double          sigma_gyro_rad_s;
    double          sigma_bias_walk;
    Eigen::Vector3d bias;

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
    std::normal_distribution<double> gyro_dist_;
    std::normal_distribution<double> unit_dist_;
};

}
