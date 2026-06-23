#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "integrators/rk45.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::dynamics;

using State6 = Eigen::Matrix<double, 6, 1>;

static State6 two_body_f(double , const State6& x) {
    State6 dxdt;
    dxdt.head<3>() = x.tail<3>();
    dxdt.tail<3>() = compute_gravity(x.head<3>());
    return dxdt;
}

static double specific_energy(const State6& x) {
    return 0.5 * x.tail<3>().squaredNorm() - k_mu / x.head<3>().norm();
}

static State6 make_iss_ic() {
    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);
    State6 x;
    x << r0, 0.0, 0.0, 0.0, v0, 0.0;
    return x;
}

TEST(RK45, EnergyConservation) {
    const State6 x0 = make_iss_ic();
    const double r0  = x0[0];
    const double T   = 2.0 * M_PI * std::sqrt(r0*r0*r0 / k_mu);

    const double E0 = specific_energy(x0);
    const State6 xf = rk45_integrate(x0, 0.0, T, two_body_f);
    const double Ef = specific_energy(xf);

    EXPECT_LT(std::abs((Ef - E0) / E0), 1e-8);
}

TEST(RK45, KeplerOrbitClosure) {
    const State6 x0 = make_iss_ic();
    const double r0  = x0[0];
    const double T   = 2.0 * M_PI * std::sqrt(r0*r0*r0 / k_mu);

    const State6 xf = rk45_integrate(x0, 0.0, T, two_body_f);

    EXPECT_NEAR(xf[0], r0,  1.0);
    EXPECT_NEAR(xf[1], 0.0, 1.0);
    EXPECT_NEAR(xf[2], 0.0, 1.0);
}
