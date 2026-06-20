#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "filters/ekf.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::filters;

// ISS-like orbit, two-body only (J2/drag off for clean Jacobian tests).
// State order (Phase 5, 12-state MEKF): [delta_theta(0-2), omega(3-5),
// r(6-8), v(9-11)] — see ekf.hpp. These tests only exercise the orbital
// block (unchanged from Phase 1); the attitude block (omega=0, q_ref =
// identity by default) stays at its torque-free fixed point throughout
// and is otherwise untouched.
static ExtendedKalmanFilter make_ekf() {
    ExtendedKalmanFilter ekf;

    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);
    ekf.x.setZero();
    ekf.x.segment<3>(6) = Eigen::Vector3d(r0, 0.0, 0.0);
    ekf.x.segment<3>(9) = Eigen::Vector3d(0.0, v0, 0.0);

    // 100 m position, 1 m/s velocity initial uncertainty
    ekf.P.setZero();
    for (int i = 6; i < 9; ++i) ekf.P(i, i) = 100.0 * 100.0;
    for (int i = 9; i < 12; ++i) ekf.P(i, i) = 1.0;

    // Process noise large enough that det(P) increase is numerically detectable
    ekf.Q.setZero();
    for (int i = 6; i < 9; ++i) ekf.Q(i, i) = 1.0;        // 1 m² per axis
    for (int i = 9; i < 12; ++i) ekf.Q(i, i) = 1e-4;       // 0.01 m/s per axis

    // 10 m GPS noise
    ekf.R.setZero();
    for (int i = 0; i < 3; ++i) ekf.R(i, i) = 10.0 * 10.0;

    // Two-body only — keep perturbations off so Jacobian test is clean
    ekf.perturb_cfg.enable_j2   = false;
    ekf.perturb_cfg.enable_drag = false;
    ekf.perturb_cfg.enable_srp  = false;

    return ekf;
}

TEST(EKF, PredictIncreasesCovariance) {
    // Without measurements, det(P) of the orbital 6x6 sub-block must
    // strictly increase (the attitude block is untouched at its
    // torque-free fixed point — omega=0 is a stationary point of Euler's
    // equation — so checking the FULL 12x12 determinant would just test
    // whether that block's det, exactly 0 since it's seeded with no
    // uncertainty, "increases" from 0, which isn't what this test is
    // about). Q_pos = 1 m² makes the increase detectable above
    // double-precision rounding.
    ExtendedKalmanFilter ekf = make_ekf();
    const Eigen::Matrix<double, 6, 6> p_orbit_before = ekf.P.block<6, 6>(6, 6);
    const double det_before = p_orbit_before.determinant();
    ekf.predict(10.0);
    const Eigen::Matrix<double, 6, 6> p_orbit_after = ekf.P.block<6, 6>(6, 6);
    const double det_after = p_orbit_after.determinant();
    EXPECT_GT(det_after, det_before);
}

TEST(EKF, UpdateDecreasesCovariance) {
    // A GPS measurement must reduce total uncertainty (trace(P) decreases).
    ExtendedKalmanFilter ekf = make_ekf();
    ekf.predict(10.0);  // advance to a realistic mid-step state first

    const double trace_before = ekf.P.trace();
    const Eigen::Vector3d z   = ekf.x.segment<3>(6);  // noiseless measurement
    ekf.update(z);

    EXPECT_LT(ekf.P.trace(), trace_before);
}

TEST(EKF, J2JacobianShiftsCovariance) {
    // J2 increases the radial Jacobian entry a_xx = 2μ/r³ + 4*factor vs 2μ/r³.
    // P(6,9) = dt*(P_pos*a_xx + P_vel): J2 case must be larger than two-body case
    // (r_x is row/col 6, v_x is row/col 9 in the 12-state layout — see header doc).
    ExtendedKalmanFilter ekf_j2 = make_ekf();
    ekf_j2.perturb_cfg.enable_j2 = true;

    ExtendedKalmanFilter ekf_twobody = make_ekf();

    ekf_j2.predict(10.0);
    ekf_twobody.predict(10.0);

    EXPECT_GT(ekf_j2.P(6, 9), ekf_twobody.P(6, 9));
}

TEST(EKF, JulianDateAdvances) {
    // predict() must advance julian_date by dt / seconds_per_day
    ExtendedKalmanFilter ekf = make_ekf();
    const double jd0 = ekf.julian_date;
    const double dt  = 10.0;
    ekf.predict(dt);
    EXPECT_NEAR(ekf.julian_date, jd0 + dt / k_sec_per_day, 1e-15);
}

// ──────────────────────────── Phase 5: attitude block ──────────────────────

TEST(EKF, AttitudeFixedPointAtZeroOmega) {
    // omega=0 is a stationary point of torque-free Euler's equation, and
    // delta_theta has no nonlinear dynamics of its own (header doc) — so
    // with default-constructed q_ref/omega, predict() must leave the
    // attitude block exactly at rest: omega stays 0, q_ref stays identity.
    ExtendedKalmanFilter ekf = make_ekf();
    ekf.predict(10.0);
    EXPECT_TRUE(ekf.x.head<3>().isZero());      // delta_theta
    EXPECT_TRUE(ekf.x.segment<3>(3).isZero());  // omega
    EXPECT_NEAR(ekf.q_ref.angularDistance(math::Quat::Identity()), 0.0, 1e-12);
}

TEST(EKF, ResetAttitudeErrorFoldsIntoQRefAndZeroes) {
    ExtendedKalmanFilter ekf = make_ekf();
    ekf.x.head<3>() = Eigen::Vector3d(0.01, -0.02, 0.03);  // pretend an update posterior
    const math::Quat q_ref_before = ekf.q_ref;

    ekf.reset_attitude_error();

    EXPECT_TRUE(ekf.x.head<3>().isZero());
    // q_ref must have actually rotated (not stayed at its prior value).
    EXPECT_GT(ekf.q_ref.angularDistance(q_ref_before), 1e-4);
    EXPECT_NEAR(ekf.q_ref.norm(), 1.0, 1e-12);
}
