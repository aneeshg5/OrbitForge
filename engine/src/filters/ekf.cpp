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
    : julian_date(orbitforge::k_j2000_jd),
      q_ref(math::Quat::Identity())
{
    x.setZero();
    P.setIdentity();
    Q.setZero();
    R.setZero();
    H.setZero();
    H.block<3, 3>(0, 6).setIdentity();  // GPS-shaped default: position block, columns 6-8
}

void ExtendedKalmanFilter::predict(double dt) {
    const Eigen::Vector3d omega_hat = x.segment<3>(3);
    const Eigen::Vector3d r = x.segment<3>(6);

    // Build continuous Jacobian F (12x12), block-diagonal between attitude
    // and orbital blocks (math.md §7.3 — no cross-coupling modeled).
    Eigen::Matrix<double, 12, 12> F = Eigen::Matrix<double, 12, 12>::Zero();
    F.block<3, 3>(0, 0) = -math::skew(omega_hat);
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(3, 3) = dynamics::euler_jacobian(omega_hat, inertia);

    Eigen::Matrix3d da_dr = gravity_jacobian(r);
    if (perturb_cfg.enable_j2) da_dr += j2_jacobian(r);
    F.block<3, 3>(6, 9)  = Eigen::Matrix3d::Identity();
    F.block<3, 3>(9, 6)  = da_dr;

    // First-order discrete transition Φ = I + F·dt (math.md §3.1, §7.3)
    const Eigen::Matrix<double, 12, 12> Phi =
        Eigen::Matrix<double, 12, 12>::Identity() + F * dt;

    // Nonlinear propagation of [omega, r, v]; delta_theta has no physical
    // value to propagate (see header doc) and is left untouched (== 0,
    // assuming reset_attitude_error() was called after the prior tick's
    // updates). q_ref propagates via its own quaternion kinematics, using
    // the SAME omega the state carries (rigid_body.hpp's AttitudeState is
    // exactly [q_ref.coeffs(), omega], so this reuses attitude_derivative
    // unchanged from the "true" trajectory's own propagation).
    dynamics::AttitudeState att;
    att.head<4>() = q_ref.coeffs();
    att.tail<3>() = omega_hat;
    const dynamics::InertiaTensor inertia_local = inertia;
    auto att_dyn = [&inertia_local](double /*t*/, const dynamics::AttitudeState& s) {
        return dynamics::attitude_derivative(s, inertia_local);
    };
    att = rk4_step(att, 0.0, dt, att_dyn);
    q_ref = math::Quat(att[3], att[0], att[1], att[2]).normalized();
    x.segment<3>(3) = att.tail<3>();

    const double jd  = julian_date;
    const auto&  cfg = perturb_cfg;
    Eigen::Matrix<double, 6, 1> rv = x.tail<6>();
    auto orbital_dyn = [&jd, &cfg](double /*t*/, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), jd, cfg);
        return ds;
    };
    rv = rk4_step(rv, 0.0, dt, orbital_dyn);
    x.tail<6>() = rv;

    P = Phi * P * Phi.transpose() + Q;

    julian_date += dt / orbitforge::k_sec_per_day;
}

void ExtendedKalmanFilter::update(const Eigen::Matrix<double, 3, 1>& z) {
    const Eigen::Matrix<double, 3, 3>  S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, 12, 3> K = P * H.transpose() * S.inverse();

    x = x + K * (z - H * x);
    // Joseph form: P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ — maintains symmetry and PD.
    const Eigen::Matrix<double, 12, 12> IKH =
        Eigen::Matrix<double, 12, 12>::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();
}

void ExtendedKalmanFilter::reset_attitude_error() {
    const Eigen::Vector3d delta_theta = x.head<3>();
    q_ref = (q_ref * math::quat_exp(delta_theta)).normalized();
    x.head<3>().setZero();
}

} // namespace orbitforge::filters
