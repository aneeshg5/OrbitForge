#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "dynamics/eom.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::dynamics;

// ISS-like circular orbit at 408 km altitude
static const double k_test_alt   = 408e3;           // m
static const double k_test_r     = k_re + k_test_alt;
static const double k_test_jd    = 2451545.0;        // J2000

TEST(ComputeGravity, CircularOrbitMagnitude) {
    // |a| should equal μ/r² to 6 significant figures
    const Eigen::Vector3d pos{k_test_r, 0.0, 0.0};
    const Eigen::Vector3d accel = compute_gravity(pos);

    const double expected = k_mu / (k_test_r * k_test_r);
    const double actual   = accel.norm();

    EXPECT_NEAR(actual, expected, expected * 1e-6);
}

TEST(ComputeGravity, DirectionTowardOrigin) {
    const Eigen::Vector3d pos{k_test_r, 0.0, 0.0};
    const Eigen::Vector3d accel = compute_gravity(pos);
    // Acceleration must point in -x direction
    EXPECT_LT(accel.x(), 0.0);
    EXPECT_NEAR(accel.y(), 0.0, 1e-20);
    EXPECT_NEAR(accel.z(), 0.0, 1e-20);
}

TEST(ComputeJ2, NonZeroWhenEnabled) {
    const Eigen::Vector3d pos{k_test_r, 0.0, 1e5}; // off-equatorial
    const Eigen::Vector3d a_j2 = compute_j2(pos);
    EXPECT_GT(a_j2.norm(), 0.0);
}

TEST(ComputeAcceleration, J2ZeroWhenDisabled) {
    const Eigen::Vector3d pos{k_test_r, 0.0, 1e5};
    const Eigen::Vector3d vel{0.0, 7700.0, 0.0};

    PerturbationConfig cfg_with_j2;
    cfg_with_j2.enable_j2   = true;
    cfg_with_j2.enable_drag = false;
    cfg_with_j2.enable_srp  = false;

    PerturbationConfig cfg_no_j2;
    cfg_no_j2.enable_j2   = false;
    cfg_no_j2.enable_drag = false;
    cfg_no_j2.enable_srp  = false;

    const Eigen::Vector3d a_with = compute_acceleration(pos, vel, k_test_jd, cfg_with_j2);
    const Eigen::Vector3d a_no   = compute_acceleration(pos, vel, k_test_jd, cfg_no_j2);

    // J2 should contribute a non-zero difference
    EXPECT_GT((a_with - a_no).norm(), 0.0);
}

TEST(ComputeDrag, ZeroAboveCeiling) {
    // 1200 km — well above the 1000 km model ceiling
    const double high_r = k_re + 1200e3;
    const Eigen::Vector3d pos{high_r, 0.0, 0.0};
    const Eigen::Vector3d vel{0.0, 7000.0, 0.0};

    const Eigen::Vector3d a_drag = compute_drag(pos, vel, 2.2, 0.01);
    EXPECT_EQ(a_drag.norm(), 0.0);
}

TEST(ComputeDrag, NonZeroInLEO) {
    const Eigen::Vector3d pos{k_test_r, 0.0, 0.0};
    const Eigen::Vector3d vel{0.0, 7700.0, 0.0};

    const Eigen::Vector3d a_drag = compute_drag(pos, vel, 2.2, 0.01);
    EXPECT_GT(a_drag.norm(), 0.0);
}
