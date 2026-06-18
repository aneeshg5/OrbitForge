#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace orbitforge::monte_carlo {

// Native thread count for the Monte Carlo run distribution. Matches the
// WASM build's PTHREAD_POOL_SIZE=4 (scripts/build_wasm.sh) so native and
// WASM behavior are the same shape.
constexpr size_t k_mc_threads = 4;

enum class FilterKind { kf, ekf, ukf };

// Configuration for one Monte Carlo campaign. The true trajectory always
// propagates via two-body RK4 with process noise w~N(0,Q) injected each
// step to match the filter's own Q — the construction that makes NEES
// well-defined for an otherwise-deterministic system (math.md §6). The
// measurement is a direct noisy ECI position read (z = r_true + N(0,
// sigma_gps^2 I)), matching test_filter_consistency.cpp's already-validated
// setup, not the full ECEF-rotated GPS model the live single-scenario
// Simulation class (wasm_api.cpp) uses — that distinction doesn't matter
// for a statistical consistency campaign and keeps this component testable
// against the Phase 1 NEES result directly.
struct MCConfig {
    size_t n_runs  = 100;
    int    n_steps = 500;
    double dt      = 10.0;

    double gps_sigma = 10.0;  // measurement noise std dev [m]
    double q_pos      = 1.0;   // per-step process noise std dev, position [m]
    double q_vel      = 0.01;  // per-step process noise std dev, velocity [m/s]
    double p0_pos      = 100.0; // initial filter position std dev [m]
    double p0_vel      = 1.0;   // initial filter velocity std dev [m/s]

    FilterKind filter = FilterKind::ekf;
    unsigned   seed   = 42;

    Eigen::Matrix<double, 6, 1> x0;  // nominal true initial state [r; v], ECI
};

struct MCStats {
    std::vector<double> rms_pos;  // size n_steps, meters
    std::vector<double> rms_vel;  // size n_steps, m/s
    std::vector<double> nees;     // size n_steps, per-step average NEES across runs
    std::vector<double> nis;      // size n_steps, per-step average NIS across runs

    // Final-step position error norm |r_true - r_hat| per run, meters.
    // Order matches run index (seed = cfg.seed + run*7919 + 1), not thread
    // completion order. This is the raw per-run data the per-step arrays
    // above average away — needed for a real final-error histogram in the
    // Monte Carlo UI panel rather than a synthetic one.
    std::vector<double> final_pos_err;  // size n_runs, meters
};

// Runs cfg.n_runs independent realizations of the configured filter,
// distributing runs across a k_mc_threads-thread pool as contiguous slices
// (no work-stealing — runs are equal cost). Returns per-step statistics
// merged across all runs and threads.
MCStats run_monte_carlo(const MCConfig& cfg);

// Wilson-Hilferty chi-squared quantile approximation (math.md §6):
//   chi2(dof, p) ~= dof * (1 - 2/(9*dof) + z_p*sqrt(2/(9*dof)))^3
// where z_p is the standard normal quantile for probability p, computed by
// bisection on the exact CDF (0.5*erfc(-z/sqrt(2))) rather than a
// transcribed rational-approximation formula, so there is no coefficient
// to get wrong. Used to compute the 95% NEES/NIS consistency bounds for
// arbitrary N.
double chi_squared_quantile(double p, double dof);

// 95% NEES/NIS consistency bounds for a campaign of n_runs:
//   NEES: chi2(6*n_runs, 0.025)/n_runs .. chi2(6*n_runs, 0.975)/n_runs
//   NIS:  chi2(3*n_runs, 0.025)/n_runs .. chi2(3*n_runs, 0.975)/n_runs
// (6 = state dim, 3 = GPS measurement dim). Constant across the campaign,
// not per-step, since n_runs doesn't change mid-run.
struct ConsistencyBounds {
    double lower = 0.0;
    double upper = 0.0;
};
ConsistencyBounds nees_bounds(size_t n_runs);
ConsistencyBounds nis_bounds(size_t n_runs);

}  // namespace orbitforge::monte_carlo
