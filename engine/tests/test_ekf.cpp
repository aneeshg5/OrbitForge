#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "filters/ekf.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::filters;

static ExtendedKalmanFilter make_ekf() {
    ExtendedKalmanFilter ekf;

    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);
    ekf.x.setZero();
    ekf.x.segment<3>(6) = Eigen::Vector3d(r0, 0.0, 0.0);
    ekf.x.segment<3>(9) = Eigen::Vector3d(0.0, v0, 0.0);

    ekf.P.setZero();
    for (int i = 6; i < 9; ++i) ekf.P(i, i) = 100.0 * 100.0;
    for (int i = 9; i < 12; ++i) ekf.P(i, i) = 1.0;

    ekf.Q.setZero();
    for (int i = 6; i < 9; ++i) ekf.Q(i, i) = 1.0;
    for (int i = 9; i < 12; ++i) ekf.Q(i, i) = 1e-4;

    ekf.R.setZero();
    for (int i = 0; i < 3; ++i) ekf.R(i, i) = 10.0 * 10.0;

    ekf.perturb_cfg.enable_j2   = false;
    ekf.perturb_cfg.enable_drag = false;
    ekf.perturb_cfg.enable_srp  = false;

    return ekf;
}

TEST(EKF, PredictIncreasesCovariance) {
    ExtendedKalmanFilter ekf = make_ekf();
    const Eigen::Matrix<double, 6, 6> p_orbit_before = ekf.P.block<6, 6>(6, 6);
    const double det_before = p_orbit_before.determinant();
    ekf.predict(10.0);
    const Eigen::Matrix<double, 6, 6> p_orbit_after = ekf.P.block<6, 6>(6, 6);
    const double det_after = p_orbit_after.determinant();
    EXPECT_GT(det_after, det_before);
}

TEST(EKF, UpdateDecreasesCovariance) {
    ExtendedKalmanFilter ekf = make_ekf();
    ekf.predict(10.0);

    const double trace_before = ekf.P.trace();
    const Eigen::Vector3d z   = ekf.x.segment<3>(6);
    ekf.update(z);

    EXPECT_LT(ekf.P.trace(), trace_before);
}

TEST(EKF, J2JacobianShiftsCovariance) {
    ExtendedKalmanFilter ekf_j2 = make_ekf();
    ekf_j2.perturb_cfg.enable_j2 = true;

    ExtendedKalmanFilter ekf_twobody = make_ekf();

    ekf_j2.predict(10.0);
    ekf_twobody.predict(10.0);

    EXPECT_GT(ekf_j2.P(6, 9), ekf_twobody.P(6, 9));
}

TEST(EKF, JulianDateAdvances) {
    ExtendedKalmanFilter ekf = make_ekf();
    const double jd0 = ekf.julian_date;
    const double dt  = 10.0;
    ekf.predict(dt);
    EXPECT_NEAR(ekf.julian_date, jd0 + dt / k_sec_per_day, 1e-15);
}

TEST(EKF, AttitudeFixedPointAtZeroOmega) {
    ExtendedKalmanFilter ekf = make_ekf();
    ekf.predict(10.0);
    EXPECT_TRUE(ekf.x.head<3>().isZero());
    EXPECT_TRUE(ekf.x.segment<3>(3).isZero());
    EXPECT_NEAR(ekf.q_ref.angularDistance(math::Quat::Identity()), 0.0, 1e-12);
}

TEST(EKF, ResetAttitudeErrorFoldsIntoQRefAndZeroes) {
    ExtendedKalmanFilter ekf = make_ekf();
    ekf.x.head<3>() = Eigen::Vector3d(0.01, -0.02, 0.03);
    const math::Quat q_ref_before = ekf.q_ref;

    ekf.reset_attitude_error();

    EXPECT_TRUE(ekf.x.head<3>().isZero());
    EXPECT_GT(ekf.q_ref.angularDistance(q_ref_before), 1e-4);
    EXPECT_NEAR(ekf.q_ref.norm(), 1.0, 1e-12);
}

// math.md §7.3.
TEST(EKF, ResetComposesCorrectionInBodyFrameRightMultiply) {
    ExtendedKalmanFilter ekf = make_ekf();
    const math::Quat q_ref_before(Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()));
    ekf.q_ref = q_ref_before;
    const Eigen::Vector3d delta_theta(0.02, -0.01, 0.015);
    ekf.x.head<3>() = delta_theta;

    ekf.reset_attitude_error();

    const Eigen::Matrix3d R_correction = Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized()).toRotationMatrix();
    const Eigen::Matrix3d R_expected = q_ref_before.toRotationMatrix() * R_correction;

    EXPECT_TRUE(ekf.q_ref.toRotationMatrix().isApprox(R_expected, 1e-9));
}
