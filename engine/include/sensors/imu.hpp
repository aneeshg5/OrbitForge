#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace orbitforge::sensors {

// math.md §5.2.
class ImuSensor {
public:
    double         sigma_acc_m_s2;
    double         sigma_bias_walk;
    Eigen::Vector3d bias;

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
    std::normal_distribution<double> acc_dist_;
    std::normal_distribution<double> unit_dist_;
};

}
