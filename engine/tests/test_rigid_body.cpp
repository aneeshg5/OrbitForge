#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "dynamics/rigid_body.hpp"
#include "integrators/rk4.hpp"
#include "math/quaternion.hpp"

using namespace orbitforge;
using namespace orbitforge::dynamics;

namespace {

const InertiaTensor k_test_inertia{1.0, 2.0, 3.0};

AttitudeState make_initial_state(const Eigen::Vector3d& omega0) {
    AttitudeState x;
    x << 0.0, 0.0, 0.0, 1.0,
         omega0.x(), omega0.y(), omega0.z();
    return x;
}

double angular_momentum_norm(const AttitudeState& x, const InertiaTensor& inertia) {
    const Eigen::Vector3d omega = x.tail<3>();
    const Eigen::Vector3d i_omega(inertia.ix * omega.x(), inertia.iy * omega.y(), inertia.iz * omega.z());
    return i_omega.norm();
}

double kinetic_energy(const AttitudeState& x, const InertiaTensor& inertia) {
    const Eigen::Vector3d omega = x.tail<3>();
    return 0.5 * (inertia.ix * omega.x() * omega.x() +
                  inertia.iy * omega.y() * omega.y() +
                  inertia.iz * omega.z() * omega.z());
}

AttitudeState propagate(const AttitudeState& x0, const InertiaTensor& inertia, double T, double h) {
    auto f = [&inertia](double , const AttitudeState& s) {
        return attitude_derivative(s, inertia);
    };

    AttitudeState x = x0;
    double t = 0.0;
    while (t + h <= T) {
        x = rk4_step(x, t, h, f);
        const math::Quat q(x[3], x[0], x[1], x[2]);
        x.head<4>() = q.normalized().coeffs();
        t += h;
    }
    if (T - t > 1e-12) {
        x = rk4_step(x, t, T - t, f);
        const math::Quat q(x[3], x[0], x[1], x[2]);
        x.head<4>() = q.normalized().coeffs();
    }
    return x;
}

}

TEST(RigidBody, AngularMomentumConservation) {
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.5, 0.3, -0.2));
    const double L0 = angular_momentum_norm(x0, k_test_inertia);

    const AttitudeState xf = propagate(x0, k_test_inertia, 10.0, 0.01);
    const double Lf = angular_momentum_norm(xf, k_test_inertia);

    EXPECT_LT(std::abs((Lf - L0) / L0), 1e-8);
}

TEST(RigidBody, KineticEnergyConservation) {
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.5, 0.3, -0.2));
    const double E0 = kinetic_energy(x0, k_test_inertia);

    const AttitudeState xf = propagate(x0, k_test_inertia, 10.0, 0.01);
    const double Ef = kinetic_energy(xf, k_test_inertia);

    EXPECT_LT(std::abs((Ef - E0) / E0), 1e-8);
}

TEST(RigidBody, QuaternionStaysNormalized) {
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.5, 0.3, -0.2));
    const AttitudeState xf = propagate(x0, k_test_inertia, 10.0, 0.01);
    const math::Quat q(xf[3], xf[0], xf[1], xf[2]);
    EXPECT_NEAR(q.norm(), 1.0, 1e-10);
}

TEST(RigidBody, SingleAxisSpinIsTorqueFreeFixedPoint) {
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.0, 0.0, 0.7));
    const AttitudeState xf = propagate(x0, k_test_inertia, 5.0, 0.01);
    EXPECT_TRUE(xf.tail<3>().isApprox(Eigen::Vector3d(0.0, 0.0, 0.7), 1e-9));
}

TEST(RigidBody, EulerJacobianMatchesFiniteDifference) {
    const Eigen::Vector3d omega(0.4, -0.6, 0.2);
    const Eigen::Matrix3d analytical = euler_jacobian(omega, k_test_inertia);

    constexpr double eps = 1e-6;
    Eigen::Matrix3d numerical;
    for (int j = 0; j < 3; ++j) {
        Eigen::Vector3d omega_plus = omega, omega_minus = omega;
        omega_plus[j]  += eps;
        omega_minus[j] -= eps;

        AttitudeState xp = make_initial_state(omega_plus);
        AttitudeState xm = make_initial_state(omega_minus);
        const Eigen::Vector3d dplus  = attitude_derivative(xp, k_test_inertia).tail<3>();
        const Eigen::Vector3d dminus = attitude_derivative(xm, k_test_inertia).tail<3>();
        numerical.col(j) = (dplus - dminus) / (2.0 * eps);
    }

    EXPECT_TRUE(analytical.isApprox(numerical, 1e-6));
}
