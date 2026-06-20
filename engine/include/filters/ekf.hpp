#pragma once

#include "filters/filter_base.hpp"
#include "dynamics/eom.hpp"
#include "dynamics/rigid_body.hpp"
#include "math/quaternion.hpp"

namespace orbitforge::filters {

// Extended Kalman Filter — 12-state multiplicative EKF (MEKF), Phase 5.
//
// State x = [delta_theta(3), omega(3), r(3), v(3)]: delta_theta is the
// MEKF attitude-error rotation vector (reset to zero by reset_attitude_error(),
// not on every predict/update individually — see math.md §7.3), omega is
// body-frame angular velocity, [r,v] is exactly Phase 1's orbital state,
// unchanged. The actual attitude estimate lives in q_ref (outside P, by
// design — a 4-parameter unit-norm quaternion has no business inside a
// covariance matrix) and is corrected by the reset step.
//
// [omega,r,v] propagate nonlinearly via RK4 in predict() (omega via
// dynamics::attitude_derivative/Euler's equation, [r,v] via the exact same
// orbital EOM as Phase 1); q_ref propagates via its own quaternion
// kinematics using the current omega estimate. delta_theta itself is not
// nonlinearly propagated — it has no physical value to track between
// resets, only its covariance block evolves (via the F matrix below).
//
// update() is measurement-agnostic: caller sets H (3x12, nonzero only in
// whichever 3-column block the measurement observes) and R before each
// call, then calls update(z) once per available measurement this tick
// (GPS position, gyro rate, and/or magnetometer field can all fire in the
// same 100 Hz tick — see Simulation::step() in wasm_api.cpp). After ALL of
// a tick's update() calls, the caller invokes reset_attitude_error() once
// — not after each individual update — since the MEKF reset is logically
// one step even when several measurements have updated delta_theta in
// sequence first.
class ExtendedKalmanFilter : public FilterBase<12, 3> {
public:
    Eigen::Matrix<double, 3, 12> H;            // measurement Jacobian (caller sets per measurement)
    dynamics::PerturbationConfig perturb_cfg;   // which forces to include in orbital propagation
    dynamics::InertiaTensor      inertia;       // principal-axis inertia tensor, attitude block
    double                       julian_date;   // current epoch; advanced by predict()
    math::Quat                   q_ref;         // reference attitude estimate (body->ECI), outside P (§6.1)

    // Initialises H = [0(theta) 0(omega) I3(r) 0(v)] (GPS-shaped default),
    // q_ref = identity, julian_date = J2000.
    ExtendedKalmanFilter();

    // Predict: nonlinearly propagates omega/r/v and q_ref via RK4 (see
    // class doc); propagates P via Phi = I + F*dt, F block-diagonal
    // between attitude and orbital blocks (math.md §7.3).
    void predict(double dt) override;
    // Update: standard Kalman equations with 3x3 S solve (identical form
    // to Phase 1 — only H/R/state dimension differ). Does NOT reset
    // delta_theta — see reset_attitude_error().
    void update(const Eigen::Matrix<double, 3, 1>& z) override;

    // MEKF reset step (math.md §7.3):
    //   q_ref <- (q_ref ⊗ quat_exp(delta_theta)).normalized();  delta_theta <- 0
    // Call once per simulation tick, after all of that tick's update() calls.
    void reset_attitude_error();
};

} // namespace orbitforge::filters
