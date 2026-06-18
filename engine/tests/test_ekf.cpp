#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "filters/ekf.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::filters;

// ISS-like orbit, two-body only (J2/drag off for clean Jacobian tests)
static ExtendedKalmanFilter make_ekf() {
    ExtendedKalmanFilter ekf;

    const double r0 = k_re + 408e3;
    const double v0 = std::sqrt(k_mu / r0);
    ekf.x << r0, 0.0, 0.0, 0.0, v0, 0.0;

    // 100 m position, 1 m/s velocity initial uncertainty
    ekf.P.setZero();
    for (int i = 0; i < 3; ++i) ekf.P(i, i) = 100.0 * 100.0;
    for (int i = 3; i < 6; ++i) ekf.P(i, i) = 1.0;

    // Process noise large enough that det(P) increase is numerically detectable
    ekf.Q.setZero();
    for (int i = 0; i < 3; ++i) ekf.Q(i, i) = 1.0;       // 1 m² per axis
    for (int i = 3; i < 6; ++i) ekf.Q(i, i) = 1e-4;      // 0.01 m/s per axis

    // 10 m GPS noise
    ekf.R.setZero();
    for (int i = 0; i < 3; ++i) ekf.R(i, i) = 10.0 * 10.0;

    // Two-body only — keep perturbations off so Jacobian test is clean
    ekf.perturb_cfg.enable_j2   = false;
    ekf.perturb_cfg.enable_drag = false;
    ekf.perturb_cfg.enable_srp  = false;

    return ekf;
}

TEST(EKF, PredictIncreasesCovariance) {
    // Without measurements, det(P) must strictly increase.
    // Q_pos = 1 m² makes the increase detectable above double-precision rounding.
    ExtendedKalmanFilter ekf = make_ekf();
    const double det_before = ekf.P.determinant();
    ekf.predict(10.0);
    EXPECT_GT(ekf.P.determinant(), det_before);
}

TEST(EKF, UpdateDecreasesCovariance) {
    // A GPS measurement must reduce total uncertainty (trace(P) decreases).
    ExtendedKalmanFilter ekf = make_ekf();
    ekf.predict(10.0);  // advance to a realistic mid-step state first

    const double trace_before = ekf.P.trace();
    const Eigen::Vector3d z   = ekf.x.head<3>();  // noiseless measurement
    ekf.update(z);

    EXPECT_LT(ekf.P.trace(), trace_before);
}

TEST(EKF, J2JacobianShiftsCovariance) {
    // J2 increases the radial Jacobian entry a_xx = 2μ/r³ + 4*factor vs 2μ/r³.
    // P(0,3) = dt*(P_pos*a_xx + P_vel): J2 case must be larger than two-body case.
    // P(0,0) is unchanged — it only depends on the top rows of Φ which are [I | dt·I].
    ExtendedKalmanFilter ekf_j2 = make_ekf();
    ekf_j2.perturb_cfg.enable_j2 = true;

    ExtendedKalmanFilter ekf_twobody = make_ekf();

    ekf_j2.predict(10.0);
    ekf_twobody.predict(10.0);

    EXPECT_GT(ekf_j2.P(0, 3), ekf_twobody.P(0, 3));
}

TEST(EKF, JulianDateAdvances) {
    // predict() must advance julian_date by dt / seconds_per_day
    ExtendedKalmanFilter ekf = make_ekf();
    const double jd0 = ekf.julian_date;
    const double dt  = 10.0;
    ekf.predict(dt);
    EXPECT_NEAR(ekf.julian_date, jd0 + dt / k_sec_per_day, 1e-15);
}
