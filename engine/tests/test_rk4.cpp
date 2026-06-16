#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "integrators/rk4.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::dynamics;

using State6 = Eigen::Matrix<double, 6, 1>;

// Two-body dynamics: ẋ = [v; a_grav] (no perturbations — orbit is exactly periodic)
static State6 two_body_f(double /*t*/, const State6& x) {
    State6 dxdt;
    dxdt.head<3>() = x.tail<3>();
    dxdt.tail<3>() = compute_gravity(x.head<3>());
    return dxdt;
}

static double specific_energy(const State6& x) {
    return 0.5 * x.tail<3>().squaredNorm() - k_mu / x.head<3>().norm();
}

// Propagate for exactly one Kepler period using h=10 s steps plus a final partial step.
static State6 propagate_one_period(const State6& x0) {
    const double r0 = x0.head<3>().norm();
    const double T  = 2.0 * M_PI * std::sqrt(r0 * r0 * r0 / k_mu);
    const double h  = 10.0;

    State6 x = x0;
    double t = 0.0;
    while (t + h <= T) {
        x = rk4_step(x, t, h, two_body_f);
        t += h;
    }
    if (T - t > 1e-10) {
        x = rk4_step(x, t, T - t, two_body_f);
    }
    return x;
}

TEST(RK4, KeplerOrbitClosure) {
    // Circular orbit at 408 km — initial state at [r, 0, 0] with tangential velocity
    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);

    State6 x0;
    x0 << r0, 0.0, 0.0, 0.0, v0, 0.0;

    const State6 xf = propagate_one_period(x0);

    // After one Kepler period, position must return within 1 m of initial
    EXPECT_NEAR(xf[0], r0,  1.0);
    EXPECT_NEAR(xf[1], 0.0, 1.0);
    EXPECT_NEAR(xf[2], 0.0, 1.0);
}

TEST(RK4, EnergyConservation) {
    // Specific orbital energy ε = v²/2 − μ/r must be constant to 1e-8 rel error
    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);

    State6 x0;
    x0 << r0, 0.0, 0.0, 0.0, v0, 0.0;

    const double E0 = specific_energy(x0);
    const State6 xf = propagate_one_period(x0);
    const double Ef = specific_energy(xf);

    EXPECT_LT(std::abs((Ef - E0) / E0), 1e-8);
}
