#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "monte_carlo/mc_runner.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::monte_carlo;

namespace {
constexpr double k_r0 = k_re + 408e3;  // ISS-like circular orbit radius
double iss_v0() { return std::sqrt(k_mu / k_r0); }

Eigen::Matrix<double, 6, 1> iss_state() {
    Eigen::Matrix<double, 6, 1> x;
    x << k_r0, 0.0, 0.0, 0.0, iss_v0(), 0.0;
    return x;
}
}  // namespace

// chi_squared_quantile must reproduce the Phase 1 hardcoded reference
// values from test_filter_consistency.cpp: chi2(600, 0.025)/100 ~= 5.35,
// chi2(600, 0.975)/100 ~= 6.69. This validates both the bisection-based
// normal quantile and the Wilson-Hilferty formula transcription at once.
TEST(ChiSquaredQuantile, MatchesKnownNeesBounds) {
    const double lower = chi_squared_quantile(0.025, 600.0) / 100.0;
    const double upper = chi_squared_quantile(0.975, 600.0) / 100.0;
    EXPECT_NEAR(lower, 5.35, 0.02);
    EXPECT_NEAR(upper, 6.69, 0.02);
}

TEST(ChiSquaredQuantile, MedianMatchesDof) {
    // chi2(dof, 0.5) ~= dof for large dof (Wilson-Hilferty is exact in the limit).
    const double median = chi_squared_quantile(0.5, 600.0);
    EXPECT_NEAR(median, 600.0, 5.0);
}

// nees_bounds(100) must reproduce the same [5.35, 6.69] reference as the
// raw chi_squared_quantile(p, 6*100)/100 computation above — it's a thin
// wrapper, this confirms the dof=6*n_runs wiring is correct.
TEST(ConsistencyBounds, NeesBoundsMatchesKnownReference) {
    const ConsistencyBounds b = nees_bounds(100);
    EXPECT_NEAR(b.lower, 5.35, 0.02);
    EXPECT_NEAR(b.upper, 6.69, 0.02);
}

TEST(ConsistencyBounds, NisBoundsUsesMeasurementDof) {
    // dof = 3*n_runs, not 6*n_runs — sanity check it's a different (tighter,
    // since dof is half) bound than NEES at the same n_runs, not a copy-paste.
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

// final_pos_err must be indexed by run order (seed = cfg.seed + run*7919+1),
// not by which thread happened to finish first — verified by comparing
// against a single-run reproduction at a specific run index.
TEST(McRunner, FinalPosErrIndexedByRunOrderNotCompletionOrder) {
    MCConfig cfg;
    cfg.n_runs = 9;  // not divisible by k_mc_threads(4) — uneven slices
    cfg.n_steps = 20;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;
    cfg.seed = 42;

    const MCStats a = run_monte_carlo(cfg);

    // Reproduce run index 5 in isolation: a 1-run campaign's internal
    // run_seed formula is (seed + 0*7919 + 1), so setting seed = cfg.seed +
    // 5*7919 makes it equal run 5's formula (cfg.seed + 5*7919 + 1) under
    // the original 9-run campaign.
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

// Run count not evenly divisible by k_mc_threads (4) — some threads get
// zero or fewer runs than others. Must not crash, and must still produce
// statistics from the runs that were assigned.
TEST(McRunner, HandlesRunCountNotDivisibleByThreadCount) {
    MCConfig cfg;
    cfg.n_runs = 3;  // < k_mc_threads
    cfg.n_steps = 20;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;

    const MCStats stats = run_monte_carlo(cfg);
    EXPECT_EQ(stats.nees.size(), 20u);
    for (double v : stats.nees) EXPECT_GT(v, 0.0);
}

// mc_progress_counter() lets a poller on another thread report "N of
// n_runs done" while run_monte_carlo() is still blocking its caller (see
// wasm_api.cpp's get_mc_progress_ptr()). By the time run_monte_carlo()
// returns, every individual realization across all k_mc_threads slices
// must have incremented it exactly once — not dropped (a thread's slice
// silently skipped) and not double-counted (the same run incrementing
// twice), either of which would make the live readout under- or
// over-shoot n_runs and never settle.
TEST(McRunner, ProgressCounterReachesNRunsExactly) {
    MCConfig cfg;
    cfg.n_runs = 17;  // not divisible by k_mc_threads(4) — uneven slices
    cfg.n_steps = 20;
    cfg.x0 = iss_state();
    cfg.filter = FilterKind::ekf;

    run_monte_carlo(cfg);
    EXPECT_EQ(mc_progress_counter().load(), 17u);
}

// A second, smaller campaign must not start from the first campaign's
// leftover count — run_monte_carlo() resets the counter to 0 on entry, not
// just at construction, so a poller watching a fresh campaign sees it
// climb from 0, not start already-elevated or overshoot the new n_runs.
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

// End-to-end reproduction of the Phase 1 NEES consistency result
// (test_filter_consistency.cpp), but now driven through the threaded
// mc_runner.cpp path with the general chi_squared_quantile() bounds
// instead of the hardcoded [5.35, 6.69] constants. Same parameters:
// N=100, 500 steps, dt=10s, ISS circular orbit, GPS sigma=10m,
// Q_pos=1m, Q_vel=0.01 m/s.
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
