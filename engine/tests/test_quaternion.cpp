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
    // Exercises the exact axis-angle branch (not the small-angle fallback)
    // at an angle well outside first-order territory.
    const Eigen::Vector3d delta_theta = Eigen::Vector3d(1.0, -0.5, 0.3).normalized() * 2.0;
    const Quat q = quat_exp(delta_theta);
    const Eigen::Vector3d recovered = quat_log(q);
    EXPECT_TRUE(recovered.isApprox(delta_theta, 1e-9));
}

TEST(Quaternion, KinematicsMatchesKnownConstantSpinRate) {
    // For constant omega = (0,0,wz) about a body initially aligned with
    // ECI (q = identity), q̇ at t=0 must match the analytical derivative of
    // a pure z-axis rotation: q(t) = [cos(wz*t/2), 0,0,sin(wz*t/2)], so
    // q̇(0) = [0,0,0, wz/2] in (w,x,y,z) order — coeffs() order (x,y,z,w)
    // puts that last.
    const double wz = 0.3;
    const Quat q = Quat::Identity();
    const Eigen::Vector4d qdot = quat_kinematics_coeffs_dot(q, Eigen::Vector3d(0.0, 0.0, wz));
    Eigen::Vector4d expected;
    expected << 0.0, 0.0, wz / 2.0, 0.0;  // (x,y,z,w) — z carries it, w stays 0
    EXPECT_TRUE(qdot.isApprox(expected, 1e-12));
}

TEST(Quaternion, KinematicsIntegrationProducesExpectedRotation) {
    // Integrate q̇ = ½q⊗[0,ω] for constant omega=(0,0,wz) over time T via
    // tiny manual Euler steps (independent of rk4_step, to cross-check it)
    // and confirm the rotation angle advances by wz*T, recovered via quat_log.
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
