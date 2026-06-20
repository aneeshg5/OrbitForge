#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "dynamics/rigid_body.hpp"
#include "integrators/rk4.hpp"
#include "math/quaternion.hpp"

using namespace orbitforge;
using namespace orbitforge::dynamics;

namespace {

// Asymmetric inertia tensor (all three principal moments distinct) so the
// tests exercise genuine gyroscopic coupling, not a degenerate single-axis
// spin that would conserve momentum/energy trivially even with a buggy
// cross-coupling term.
const InertiaTensor k_test_inertia{/*ix=*/1.0, /*iy=*/2.0, /*iz=*/3.0};

AttitudeState make_initial_state(const Eigen::Vector3d& omega0) {
    AttitudeState x;
    x << 0.0, 0.0, 0.0, 1.0,            // q = identity, coeffs() order (x,y,z,w)
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

// Propagates via RK4 for duration T at fixed step h, renormalizing the
// quaternion block after every step (q has no effect on omega's dynamics
// for torque-free motion, but a real "true trajectory" integrator must
// still control its drift off the unit-quaternion manifold — see
// dynamics/rigid_body.hpp's AttitudeState doc and CLAUDE.md §6).
AttitudeState propagate(const AttitudeState& x0, const InertiaTensor& inertia, double T, double h) {
    auto f = [&inertia](double /*t*/, const AttitudeState& s) {
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

} // namespace

TEST(RigidBody, AngularMomentumConservation) {
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.5, 0.3, -0.2));
    const double L0 = angular_momentum_norm(x0, k_test_inertia);

    const AttitudeState xf = propagate(x0, k_test_inertia, /*T=*/10.0, /*h=*/0.01);
    const double Lf = angular_momentum_norm(xf, k_test_inertia);

    EXPECT_LT(std::abs((Lf - L0) / L0), 1e-8);
}

TEST(RigidBody, KineticEnergyConservation) {
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.5, 0.3, -0.2));
    const double E0 = kinetic_energy(x0, k_test_inertia);

    const AttitudeState xf = propagate(x0, k_test_inertia, /*T=*/10.0, /*h=*/0.01);
    const double Ef = kinetic_energy(xf, k_test_inertia);

    EXPECT_LT(std::abs((Ef - E0) / E0), 1e-8);
}

TEST(RigidBody, QuaternionStaysNormalized) {
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.5, 0.3, -0.2));
    const AttitudeState xf = propagate(x0, k_test_inertia, /*T=*/10.0, /*h=*/0.01);
    const math::Quat q(xf[3], xf[0], xf[1], xf[2]);
    EXPECT_NEAR(q.norm(), 1.0, 1e-10);
}

TEST(RigidBody, SingleAxisSpinIsTorqueFreeFixedPoint) {
    // Spin purely about a principal axis: Euler's equation degenerates to
    // omega_dot = 0 exactly (no gyroscopic coupling when only one
    // component is nonzero), so omega must stay constant.
    const AttitudeState x0 = make_initial_state(Eigen::Vector3d(0.0, 0.0, 0.7));
    const AttitudeState xf = propagate(x0, k_test_inertia, /*T=*/5.0, /*h=*/0.01);
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
