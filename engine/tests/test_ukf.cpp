#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "filters/ukf.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::filters;

// ISS-like circular orbit, two-body only
static UnscentedKalmanFilter make_ukf() {
    UnscentedKalmanFilter ukf;

    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);
    ukf.x << r0, 0.0, 0.0, 0.0, v0, 0.0;

    // 100 m position, 1 m/s velocity initial uncertainty
    ukf.P.setZero();
    for (int i = 0; i < 3; ++i) ukf.P(i, i) = 100.0 * 100.0;
    for (int i = 3; i < 6; ++i) ukf.P(i, i) = 1.0;

    // Initialise S = chol(P) so the SR form is consistent with P from the start
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(ukf.P);
    ukf.S = llt.matrixL();

    // Process noise (1 m² / 0.01 (m/s)²) large enough that covariance growth is visible
    ukf.Q.setZero();
    for (int i = 0; i < 3; ++i) ukf.Q(i, i) = 1.0;
    for (int i = 3; i < 6; ++i) ukf.Q(i, i) = 1e-4;

    // 10 m GPS noise
    ukf.R.setZero();
    for (int i = 0; i < 3; ++i) ukf.R(i, i) = 10.0 * 10.0;

    // Two-body only for clean, reproducible dynamics
    ukf.perturb_cfg.enable_j2   = false;
    ukf.perturb_cfg.enable_drag = false;
    ukf.perturb_cfg.enable_srp  = false;

    return ukf;
}

TEST(UKF, SigmaPointSymmetry) {
    // The weighted mean of sigma points must equal x exactly.
    // For symmetric pairs chi_i = x ± gamma·S[:,i] and equal weights W_mi,
    // the positive and negative deviations cancel and the sum reduces to x.
    UnscentedKalmanFilter ukf = make_ukf();

    constexpr double alpha  = 1e-3;
    constexpr double kappa  = 0.0;
    const double lambda = alpha*alpha*(6.0 + kappa) - 6.0;
    const double nl     = 6.0 + lambda;
    const double gamma  = std::sqrt(nl);
    const double W_m0   = lambda / nl;
    const double W_mi   = 0.5 / nl;

    // Build sigma points (same formula as ukf.cpp)
    Eigen::Matrix<double, 6, 13> chi;
    chi.col(0) = ukf.x;
    for (int i = 0; i < 6; ++i) {
        chi.col(i + 1)  = ukf.x + gamma * ukf.S.col(i);
        chi.col(i + 7)  = ukf.x - gamma * ukf.S.col(i);
    }

    // Weighted mean must equal x to floating-point precision
    Eigen::Matrix<double, 6, 1> mean = W_m0 * chi.col(0);
    for (int i = 1; i < 13; ++i) mean += W_mi * chi.col(i);

    EXPECT_TRUE(mean.isApprox(ukf.x, 1e-9));
}

TEST(SRUKF, CholeskyRemainsValid) {
    // All diagonal elements of S must stay strictly positive through predict+update cycles.
    UnscentedKalmanFilter ukf = make_ukf();

    for (int step = 0; step < 10; ++step) {
        ukf.predict(10.0);

        // Simulate a noisy GPS measurement near the true position
        const Eigen::Vector3d z = ukf.x.head<3>() +
            Eigen::Vector3d(1.0 * step, -0.5 * step, 0.3 * step);
        ukf.update(z);

        for (int i = 0; i < 6; ++i) {
            EXPECT_GT(ukf.S(i, i), 0.0)
                << "S diagonal broken at step " << step << ", index " << i;
        }
    }
}

TEST(UKF, PredictIncreasesCovariance) {
    // Without measurements, process noise must drive trace(P) upward.
    UnscentedKalmanFilter ukf = make_ukf();
    const double trace_before = ukf.P.trace();
    ukf.predict(10.0);
    EXPECT_GT(ukf.P.trace(), trace_before);
}

TEST(UKF, UpdateDecreasesCovariance) {
    // A GPS measurement must reduce total uncertainty (trace(P) decreases).
    UnscentedKalmanFilter ukf = make_ukf();
    ukf.predict(10.0);  // advance to a realistic mid-step state first

    const double trace_before = ukf.P.trace();
    const Eigen::Vector3d z   = ukf.x.head<3>();  // noiseless measurement
    ukf.update(z);

    EXPECT_LT(ukf.P.trace(), trace_before);
}
