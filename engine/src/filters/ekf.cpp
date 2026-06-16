#include "filters/ekf.hpp"
#include "integrators/rk4.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

#include <cmath>

namespace orbitforge::filters {

namespace {

// ∂a_grav/∂r = -μ/|r|³ · (I₃ - 3·r̂·r̂ᵀ)  (math.md §3.1)
Eigen::Matrix3d gravity_jacobian(const Eigen::Vector3d& r) {
    const double r_norm = r.norm();
    const double r3     = r_norm * r_norm * r_norm;
    const Eigen::Vector3d r_hat = r / r_norm;
    return (-orbitforge::k_mu / r3) *
           (Eigen::Matrix3d::Identity() - 3.0 * r_hat * r_hat.transpose());
}

// ∂a_J2/∂r — 9 entries, symmetric (math.md §3.2)
Eigen::Matrix3d j2_jacobian(const Eigen::Vector3d& r) {
    const double x   = r.x(), y = r.y(), z = r.z();
    const double r2  = r.squaredNorm();
    const double r4  = r2 * r2;
    const double r5  = r4 * std::sqrt(r2);
    const double z2r2 = z * z / r2;          // (z/|r|)²

    const double factor =
        1.5 * orbitforge::k_j2 * orbitforge::k_mu *
        orbitforge::k_re * orbitforge::k_re / r5;

    Eigen::Matrix3d J;
    J(0, 0) = factor * (5.0*(x*x + z*z)/r2 - 1.0 - 35.0*x*x*z*z/r4);
    J(1, 1) = factor * (5.0*(y*y + z*z)/r2 - 1.0 - 35.0*y*y*z*z/r4);
    J(2, 2) = factor * (-3.0 + 30.0*z2r2 - 35.0*z2r2*z2r2);
    J(0, 1) = J(1, 0) = factor * 5.0*x*y/r2 * (1.0 - 7.0*z2r2);
    J(0, 2) = J(2, 0) = factor * 5.0*x*z/r2 * (3.0 - 7.0*z2r2);
    J(1, 2) = J(2, 1) = factor * 5.0*y*z/r2 * (3.0 - 7.0*z2r2);
    return J;
}

} // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter()
    : julian_date(orbitforge::k_j2000_jd)
{
    x.setZero();
    P.setIdentity();
    Q.setZero();
    R.setZero();
    H.setZero();
    H.block<3, 3>(0, 0).setIdentity();
}

void ExtendedKalmanFilter::predict(double dt) {
    const Eigen::Vector3d r = x.head<3>();

    // Build continuous Jacobian F (gravity + J2; drag ∂a/∂v omitted — see header)
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d da_dr = gravity_jacobian(r);
    if (perturb_cfg.enable_j2) da_dr += j2_jacobian(r);
    F.block<3, 3>(3, 0) = da_dr;

    // First-order discrete transition Φ = I + F·dt (math.md §3.1)
    const Eigen::Matrix<double, 6, 6> Phi =
        Eigen::Matrix<double, 6, 6>::Identity() + F * dt;

    // Nonlinear state propagation via RK4 (captures drag, SRP, exact gravity)
    const double jd    = julian_date;
    const auto&  cfg   = perturb_cfg;

    auto dynamics = [&jd, &cfg](double /*t*/, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), jd, cfg);
        return ds;
    };

    x = rk4_step(x, 0.0, dt, dynamics);
    P = Phi * P * Phi.transpose() + Q;

    julian_date += dt / orbitforge::k_sec_per_day;
}

void ExtendedKalmanFilter::update(const Eigen::Matrix<double, 3, 1>& z) {
    const Eigen::Matrix<double, 3, 3> S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();

    x = x + K * (z - H * x);
    // Joseph form: P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ  — maintains symmetry and PD.
    const Eigen::Matrix<double, 6, 6> IKH =
        Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();
}

} // namespace orbitforge::filters
