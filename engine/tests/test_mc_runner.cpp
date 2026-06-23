#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "monte_carlo/mc_runner.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::monte_carlo;

namespace {
constexpr double k_r0 = k_re + 408e3;
double iss_v0() { return std::sqrt(k_mu / k_r0); }

Eigen::Matrix<double, 6, 1> iss_state() {
    Eigen::Matrix<double, 6, 1> x;
    x << k_r0, 0.0, 0.0, 0.0, iss_v0(), 0.0;
    return x;
}
}

TEST(ChiSquaredQuantile, MatchesKnownNeesBounds) {
    const double lower = chi_squared_quantile(0.025, 600.0) / 100.0;
    const double upper = chi_squared_quantile(0.975, 600.0) / 100.0;
    EXPECT_NEAR(lower, 5.35, 0.02);
    EXPECT_NEAR(upper, 6.69, 0.02);
}

TEST(ChiSquaredQuantile, MedianMatchesDof) {
    const double median = chi_squared_quantile(0.5, 600.0);
    EXPECT_NEAR(median, 600.0, 5.0);
}

TEST(ConsistencyBounds, NeesBoundsMatchesKnownReference) {
    const ConsistencyBounds b = nees_bounds(100);
    EXPECT_NEAR(b.lower, 5.35, 0.02);
    EXPECT_NEAR(b.upper, 6.69, 0.02);
}

TEST(ConsistencyBounds, NisBoundsUsesMeasurementDof) {
    const ConsistencyBounds nees = nees_bounds(100);
    const ConsistencyBounds nis = nis_bounds(100);
    EXPECT_LT(nis.upper, nees.upper);
    EXPECT_GT(nis.lower, 0.0);
}

TEST(McRunner, OutputArraysAreCorrectlySized) {
    MCConfig cfg;
    cfg.n_runs = 8;
    cfg.n_steps = 50;
    cfg.dt = 10.0;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;

    const MCStats stats = run_monte_carlo(cfg);
    EXPECT_EQ(stats.rms_pos.size(), 50u);
    EXPECT_EQ(stats.rms_vel.size(), 50u);
    EXPECT_EQ(stats.nees.size(), 50u);
    EXPECT_EQ(stats.nis.size(), 50u);
    EXPECT_EQ(stats.final_pos_err.size(), 8u);
    for (double v : stats.rms_pos) EXPECT_GE(v, 0.0);
    for (double v : stats.nees) EXPECT_GT(v, 0.0);
    for (double v : stats.final_pos_err) EXPECT_GE(v, 0.0);
}

TEST(McRunner, FinalPosErrIndexedByRunOrderNotCompletionOrder) {
    MCConfig cfg;
    cfg.n_runs = 9;
    cfg.n_steps = 20;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;
    cfg.seed = 42;

    const MCStats a = run_monte_carlo(cfg);

    MCConfig single = cfg;
    single.n_runs = 1;
    single.seed = cfg.seed + 5 * 7919;
    const MCStats b = run_monte_carlo(single);

    EXPECT_DOUBLE_EQ(a.final_pos_err[5], b.final_pos_err[0]);
}

TEST(McRunner, DeterministicGivenSameSeed) {
    MCConfig cfg;
    cfg.n_runs = 16;
    cfg.n_steps = 30;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;
    cfg.seed = 1234;

    const MCStats a = run_monte_carlo(cfg);
    const MCStats b = run_monte_carlo(cfg);
    for (size_t i = 0; i < a.nees.size(); ++i) {
        EXPECT_DOUBLE_EQ(a.nees[i], b.nees[i]);
        EXPECT_DOUBLE_EQ(a.rms_pos[i], b.rms_pos[i]);
    }
}

TEST(McRunner, HandlesRunCountNotDivisibleByThreadCount) {
    MCConfig cfg;
    cfg.n_runs = 3;
    cfg.n_steps = 20;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;

    const MCStats stats = run_monte_carlo(cfg);
    EXPECT_EQ(stats.nees.size(), 20u);
    for (double v : stats.nees) EXPECT_GT(v, 0.0);
}

TEST(McRunner, ProgressCounterReachesNRunsExactly) {
    MCConfig cfg;
    cfg.n_runs = 17;
    cfg.n_steps = 20;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;

    run_monte_carlo(cfg);
    EXPECT_EQ(mc_progress_counter().load(), 17u);
}

TEST(McRunner, ProgressCounterResetsOnEachNewCampaign) {
    MCConfig cfg;
    cfg.n_steps = 20;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;

    cfg.n_runs = 12;
    run_monte_carlo(cfg);
    ASSERT_EQ(mc_progress_counter().load(), 12u);

    cfg.n_runs = 5;
    run_monte_carlo(cfg);
    EXPECT_EQ(mc_progress_counter().load(), 5u);
}

TEST(McRunner, KalmanFilterKindRunsWithoutCrashing) {
    MCConfig cfg;
    cfg.n_runs = 8;
    cfg.n_steps = 30;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::kf;

    const MCStats stats = run_monte_carlo(cfg);
    for (double v : stats.nees) EXPECT_TRUE(std::isfinite(v));
    for (double v : stats.rms_pos) EXPECT_TRUE(std::isfinite(v));
}

TEST(McRunner, UnscentedKalmanFilterKindRunsWithoutCrashing) {
    MCConfig cfg;
    cfg.n_runs = 8;
    cfg.n_steps = 30;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ukf;

    const MCStats stats = run_monte_carlo(cfg);
    for (double v : stats.nees) EXPECT_TRUE(std::isfinite(v));
    for (double v : stats.rms_pos) EXPECT_TRUE(std::isfinite(v));
}

TEST(McRunner, EkfNeesConsistencyMatchesPhase1Result) {
    MCConfig cfg;
    cfg.n_runs = 100;
    cfg.n_steps = 500;
    cfg.dt = 10.0;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;
    cfg.gps_sigma = 10.0;
    cfg.q_pos = 1.0;
    cfg.q_vel = 0.01;
    cfg.p0_pos = 100.0;
    cfg.p0_vel = 1.0;
    cfg.seed = 777;

    const MCStats stats = run_monte_carlo(cfg);

    const double lower = chi_squared_quantile(0.025, 6.0 * cfg.n_runs) / cfg.n_runs;
    const double upper = chi_squared_quantile(0.975, 6.0 * cfg.n_runs) / cfg.n_runs;

    int in_bounds = 0;
    for (double v : stats.nees) {
        if (v >= lower && v <= upper) ++in_bounds;
    }
    const double frac = static_cast<double>(in_bounds) / static_cast<double>(stats.nees.size());
    EXPECT_GE(frac, 0.90) << "NEES fraction in [" << lower << ", " << upper << "]: " << frac;
}
