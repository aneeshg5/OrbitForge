#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "filters/ukf.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::filters;

static UnscentedKalmanFilter make_ukf() {
    UnscentedKalmanFilter ukf;

    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);
    ukf.x.setZero();
    ukf.x.segment<3>(6) = Eigen::Vector3d(r0, 0.0, 0.0);
    ukf.x.segment<3>(9) = Eigen::Vector3d(0.0, v0, 0.0);

    ukf.P.setIdentity();
    for (int i = 6; i < 9; ++i) ukf.P(i, i) = 100.0 * 100.0;
    for (int i = 9; i < 12; ++i) ukf.P(i, i) = 1.0;

    Eigen::LLT<Eigen::Matrix<double, 12, 12>> llt(ukf.P);
    ukf.S = llt.matrixL();

    ukf.Q.setZero();
    for (int i = 6; i < 9; ++i) ukf.Q(i, i) = 1.0;
    for (int i = 9; i < 12; ++i) ukf.Q(i, i) = 1e-4;

    ukf.R.setZero();
    for (int i = 0; i < 3; ++i) ukf.R(i, i) = 10.0 * 10.0;

    ukf.perturb_cfg.enable_j2   = false;
    ukf.perturb_cfg.enable_drag = false;
    ukf.perturb_cfg.enable_srp  = false;

    return ukf;
}

TEST(UKF, SigmaPointSymmetry) {
    UnscentedKalmanFilter ukf = make_ukf();

    constexpr double alpha  = 1e-3;
    constexpr double kappa  = 0.0;
    constexpr int    n      = 12;
    const double lambda = alpha*alpha*(static_cast<double>(n) + kappa) - n;
    const double nl     = n + lambda;
    const double gamma  = std::sqrt(nl);
    const double W_m0   = lambda / nl;
    const double W_mi   = 0.5 / nl;

    Eigen::Matrix<double, n, 2*n+1> chi;
    chi.col(0) = ukf.x;
    for (int i = 0; i < n; ++i) {
        chi.col(i + 1)     = ukf.x + gamma * ukf.S.col(i);
        chi.col(i + n + 1) = ukf.x - gamma * ukf.S.col(i);
    }

    Eigen::Matrix<double, n, 1> mean = W_m0 * chi.col(0);
    for (int i = 1; i < 2*n+1; ++i) mean += W_mi * chi.col(i);

    EXPECT_TRUE(mean.isApprox(ukf.x, 1e-9));
}

TEST(SRUKF, CholeskyRemainsValid) {
    UnscentedKalmanFilter ukf = make_ukf();

    for (int step = 0; step < 10; ++step) {
        ukf.predict(10.0);

        const Eigen::Vector3d z = ukf.x.segment<3>(6) +
            Eigen::Vector3d(1.0 * step, -0.5 * step, 0.3 * step);
        ukf.update(z);

        for (int i = 0; i < 12; ++i) {
            EXPECT_GT(ukf.S(i, i), 0.0)
                << "S diagonal broken at step " << step << ", index " << i;
        }
    }
}

TEST(UKF, PredictIncreasesCovariance) {
    UnscentedKalmanFilter ukf = make_ukf();
    const double trace_before = ukf.P.trace();
    ukf.predict(10.0);
    EXPECT_GT(ukf.P.trace(), trace_before);
}

TEST(UKF, UpdateDecreasesCovariance) {
    UnscentedKalmanFilter ukf = make_ukf();
    ukf.predict(10.0);

    const double trace_before = ukf.P.trace();
    const Eigen::Vector3d z   = ukf.x.segment<3>(6);
    ukf.update(z);

    EXPECT_LT(ukf.P.trace(), trace_before);
}

TEST(UKF, AttitudeFixedPointAtZeroOmega) {
    UnscentedKalmanFilter ukf = make_ukf();
    ukf.predict(10.0);
    EXPECT_TRUE(ukf.x.head<3>().isApprox(Eigen::Vector3d::Zero(), 1e-9));
    EXPECT_TRUE(ukf.x.segment<3>(3).isApprox(Eigen::Vector3d::Zero(), 1e-9));
    EXPECT_NEAR(ukf.q_ref.angularDistance(math::Quat::Identity()), 0.0, 1e-9);
}

TEST(UKF, ResetAttitudeErrorFoldsIntoQRefAndZeroes) {
    UnscentedKalmanFilter ukf = make_ukf();
    ukf.x.head<3>() = Eigen::Vector3d(0.01, -0.02, 0.03);
    const math::Quat q_ref_before = ukf.q_ref;

    ukf.reset_attitude_error();

    EXPECT_TRUE(ukf.x.head<3>().isZero());
    EXPECT_GT(ukf.q_ref.angularDistance(q_ref_before), 1e-4);
    EXPECT_NEAR(ukf.q_ref.norm(), 1.0, 1e-12);
}

TEST(UKF, ResetComposesCorrectionInBodyFrameRightMultiply) {
    UnscentedKalmanFilter ukf = make_ukf();
    const math::Quat q_ref_before(Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()));
    ukf.q_ref = q_ref_before;
    const Eigen::Vector3d delta_theta(0.02, -0.01, 0.015);
    ukf.x.head<3>() = delta_theta;

    ukf.reset_attitude_error();

    const Eigen::Matrix3d R_correction = Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized()).toRotationMatrix();
    const Eigen::Matrix3d R_expected = q_ref_before.toRotationMatrix() * R_correction;

    EXPECT_TRUE(ukf.q_ref.toRotationMatrix().isApprox(R_expected, 1e-9));
}
