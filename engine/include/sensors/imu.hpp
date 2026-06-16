#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace orbitforge::sensors {

// IMU accelerometer model with bias random walk and white noise.
// z_acc = a_true + bias + N(0, sigma_acc²·I)
// bias(t+dt) = bias(t) + N(0, sigma_bias²·dt·I)
// Defaults: sigma_acc = 0.05 m/s², sigma_bias = 0.001 m/s²/√s. See math.md §5.2.
class ImuSensor {
public:
    double         sigma_acc_m_s2;   // white noise std dev [m/s²]
    double         sigma_bias_walk;  // bias spectral density [m/s²/√s]
    Eigen::Vector3d bias;             // current bias state [m/s²]

    ImuSensor(double sigma_acc  = 0.05,
              double sigma_bias = 0.001,
              unsigned seed = 42)
        : sigma_acc_m_s2(sigma_acc),
          sigma_bias_walk(sigma_bias),
          bias(Eigen::Vector3d::Zero()),
          rng_(seed),
          acc_dist_(0.0, sigma_acc),
          unit_dist_(0.0, 1.0)
    {}

    // Advances bias by a random-walk step of size sigma_bias·sqrt(dt),
    // then returns a_true + bias + N(0, sigma_acc²·I).
    Eigen::Vector3d measure(const Eigen::Vector3d& a_true, double dt) {
        if (dt > 0.0) {
            const double sb = sigma_bias_walk * std::sqrt(dt);
            bias += Eigen::Vector3d(
                sb * unit_dist_(rng_),
                sb * unit_dist_(rng_),
                sb * unit_dist_(rng_)
            );
        }
        return a_true + bias +
               Eigen::Vector3d(acc_dist_(rng_), acc_dist_(rng_), acc_dist_(rng_));
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> acc_dist_;   // N(0, sigma_acc²)
    std::normal_distribution<double> unit_dist_;  // N(0,1), scaled by sigma_bias·√dt
};

} // namespace orbitforge::sensors
