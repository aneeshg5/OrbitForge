#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "math/quaternion.hpp"

using namespace orbitforge::math;

TEST(Quaternion, SkewMatchesCrossProduct) {
    const Eigen::Vector3d v(1.0, 2.0, 3.0);
    const Eigen::Vector3d x(4.0, 5.0, 6.0);
    EXPECT_TRUE((skew(v) * x).isApprox(v.cross(x), 1e-12));
}

TEST(Quaternion, ExpAtZeroIsIdentity) {
    const Quat q = quat_exp(Eigen::Vector3d::Zero());
    EXPECT_NEAR(q.w(), 1.0, 1e-12);
    EXPECT_NEAR(q.vec().norm(), 0.0, 1e-12);
}

TEST(Quaternion, ExpLogRoundTrip) {
    const Eigen::Vector3d delta_theta(0.05, -0.12, 0.2);
    const Quat q = quat_exp(delta_theta);
    EXPECT_NEAR(q.norm(), 1.0, 1e-12);
    const Eigen::Vector3d recovered = quat_log(q);
    EXPECT_TRUE(recovered.isApprox(delta_theta, 1e-9));
}

TEST(Quaternion, ExpLogRoundTripLargeAngle) {
    const Eigen::Vector3d delta_theta = Eigen::Vector3d(1.0, -0.5, 0.3).normalized() * 2.0;
    const Quat q = quat_exp(delta_theta);
    const Eigen::Vector3d recovered = quat_log(q);
    EXPECT_TRUE(recovered.isApprox(delta_theta, 1e-9));
}

TEST(Quaternion, KinematicsMatchesKnownConstantSpinRate) {
    const double wz = 0.3;
    const Quat q = Quat::Identity();
    const Eigen::Vector4d qdot = quat_kinematics_coeffs_dot(q, Eigen::Vector3d(0.0, 0.0, wz));
    Eigen::Vector4d expected;
    expected << 0.0, 0.0, wz / 2.0, 0.0;
    EXPECT_TRUE(qdot.isApprox(expected, 1e-12));
}

TEST(Quaternion, KinematicsIntegrationProducesExpectedRotation) {
    const double wz = 0.4;
    const double T  = 1.0;
    const int    n_steps = 200000;
    const double dt = T / n_steps;

    Quat q = Quat::Identity();
    for (int i = 0; i < n_steps; ++i) {
        const Eigen::Vector4d qdot = quat_kinematics_coeffs_dot(q, Eigen::Vector3d(0.0, 0.0, wz));
        Eigen::Vector4d coeffs = q.coeffs() + dt * qdot;
        q = Quat(coeffs(3), coeffs(0), coeffs(1), coeffs(2));
        q.normalize();
    }

    const Eigen::Vector3d theta = quat_log(q);
    EXPECT_NEAR(theta.norm(), wz * T, 1e-3);
    EXPECT_TRUE(theta.normalized().isApprox(Eigen::Vector3d(0, 0, 1), 1e-6));
}
