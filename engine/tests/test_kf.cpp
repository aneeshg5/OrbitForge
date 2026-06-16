#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "filters/kf.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::filters;

// ISS-like circular orbit initial state
static KalmanFilter make_kf() {
    KalmanFilter kf;

    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);
    kf.x << r0, 0.0, 0.0, 0.0, v0, 0.0;

    // 100 m position uncertainty, 1 m/s velocity uncertainty
    kf.P.setZero();
    for (int i = 0; i < 3; ++i) kf.P(i, i) = 100.0 * 100.0;
    for (int i = 3; i < 6; ++i) kf.P(i, i) = 1.0 * 1.0;

    // Small process noise
    kf.Q.setZero();
    for (int i = 0; i < 3; ++i) kf.Q(i, i) = 1e-4;
    for (int i = 3; i < 6; ++i) kf.Q(i, i) = 1e-8;

    // 10 m GPS noise
    kf.R.setZero();
    for (int i = 0; i < 3; ++i) kf.R(i, i) = 10.0 * 10.0;

    return kf;
}

TEST(KalmanFilter, PredictIncreasesCovariance) {
    // Without measurements, total uncertainty must grow.
    // trace is used rather than det: det of a 6×6 matrix with condition number ~1e4
    // is numerically unreliable at double precision for the tiny changes per step.
    KalmanFilter kf = make_kf();
    const double trace_before = kf.P.trace();
    kf.predict(10.0);
    EXPECT_GT(kf.P.trace(), trace_before);
}

TEST(KalmanFilter, UpdateDecreasesCovariance) {
    // A GPS fix must reduce total uncertainty (trace of P decreases after update)
    KalmanFilter kf = make_kf();
    const double trace_before = kf.P.trace();

    const Eigen::Vector3d z = kf.x.head<3>();  // noiseless measurement at estimate
    kf.update(z);

    EXPECT_LT(kf.P.trace(), trace_before);
}

TEST(KalmanFilter, DefaultHIsPositionOnly) {
    // H default must select position and zero velocity columns
    KalmanFilter kf = make_kf();
    const Eigen::Matrix3d H_pos = kf.H.leftCols<3>();
    const Eigen::Matrix3d H_vel = kf.H.rightCols<3>();
    EXPECT_TRUE(H_pos.isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(H_vel.isZero());
}
