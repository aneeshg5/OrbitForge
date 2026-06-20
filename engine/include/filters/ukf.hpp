#pragma once

#include "filters/filter_base.hpp"
#include "dynamics/eom.hpp"
#include "dynamics/rigid_body.hpp"
#include "math/quaternion.hpp"

namespace orbitforge::filters {

// Unscented Kalman Filter — square-root form (SR-UKF), 12-state MEKF (Phase 5).
//
// Same state layout, q_ref design, and reset-step contract as
// ExtendedKalmanFilter (see its header doc and math.md §7.3) — only the
// predict/update mechanics differ (sigma points instead of an analytical
// Jacobian). n=12 now (was 6), so 2n+1=25 sigma points (was 13).
//
// Maintains S (lower triangular Cholesky of P) directly; P = S·Sᵀ is never
// accumulated in the hot path. Predict uses QR to form S⁻; update uses
// rank-1 Cholesky downdates (cholupdate inline, no LAPACK).
class UnscentedKalmanFilter : public FilterBase<12, 3> {
public:
    Eigen::Matrix<double, 12, 12> S;            // lower triangular: P = S·Sᵀ
    Eigen::Matrix<double, 3, 12>  H;             // measurement Jacobian (caller sets per measurement)
    dynamics::PerturbationConfig  perturb_cfg;
    dynamics::InertiaTensor       inertia;       // principal-axis inertia tensor, attitude block
    double                        julian_date;
    math::Quat                    q_ref;         // reference attitude estimate (body->ECI), outside P (§6.1)

    // Initialises H = [0(theta) 0(omega) I3(r) 0(v)], S = I12, q_ref = identity, julian_date = J2000.
    UnscentedKalmanFilter();

    // Predict: propagates 25 sigma points via RK4 (attitude block uses
    // dynamics::attitude_derivative; orbital block exactly as Phase 1),
    // reconstructs S⁻ via QR + rank-1 downdate. q_ref propagates using
    // the mean omega trajectory, same as ExtendedKalmanFilter::predict().
    void predict(double dt) override;
    // Update: standard Kalman equations with Joseph-form P refresh and S = chol(P_new).
    // Does NOT reset delta_theta — see reset_attitude_error().
    void update(const Eigen::Matrix<double, 3, 1>& z) override;

    // MEKF reset step — identical contract to ExtendedKalmanFilter's (math.md §7.3).
    // Call once per simulation tick, after all of that tick's update() calls.
    void reset_attitude_error();
};

} // namespace orbitforge::filters
