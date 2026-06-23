#include "filters/kf.hpp"
#include "constants.hpp"

#include <cmath>

namespace orbitforge::filters {

KalmanFilter::KalmanFilter() {
    x.setZero();
    P.setIdentity();
    Q.setZero();
    R.setZero();
    H.setZero();
    H.block<3, 3>(0, 0).setIdentity();
}

void KalmanFilter::predict(double dt) {
    // Gravity Jacobian ∂a/∂r = −μ/|r|³ · (I₃ − 3·r̂·r̂ᵀ)  (math.md §3.1)
    const Eigen::Vector3d r      = x.head<3>();
    const double          r_norm = r.norm();
    const double          r3     = r_norm * r_norm * r_norm;
    const Eigen::Vector3d r_hat  = r / r_norm;

    const Eigen::Matrix3d da_dr =
        (-orbitforge::k_mu / r3) *
        (Eigen::Matrix3d::Identity() - 3.0 * r_hat * r_hat.transpose());

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
    F.block<3, 3>(0, 3)           = Eigen::Matrix3d::Identity();
    F.block<3, 3>(3, 0)           = da_dr;

    const Eigen::Matrix<double, 6, 6> Phi =
        Eigen::Matrix<double, 6, 6>::Identity() + F * dt;

    x = Phi * x;
    P = Phi * P * Phi.transpose() + Q;
}

void KalmanFilter::update(const Eigen::Matrix<double, 3, 1>& z) {
    const Eigen::Matrix<double, 3, 3> S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();

    x = x + K * (z - H * x);
    P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
}

}
