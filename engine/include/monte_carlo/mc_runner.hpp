#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace orbitforge::monte_carlo {

constexpr size_t k_mc_threads = 4;

enum class FilterKind { kf, ekf, ukf };

// math.md §6.
struct MCConfig {
    size_t n_runs  = 100;
    int    n_steps = 500;
    double dt      = 10.0;

    double gps_sigma = 10.0;
    double q_pos      = 1.0;
    double q_vel      = 0.01;
    double p0_pos      = 100.0;
    double p0_vel      = 1.0;

    FilterKind filter = FilterKind::ekf;
    unsigned   seed   = 42;

    Eigen::Matrix<double, 6, 1> x0;
};

struct MCStats {
    std::vector<double> rms_pos;
    std::vector<double> rms_vel;
    std::vector<double> nees;
    std::vector<double> nis;
    std::vector<double> final_pos_err;
};

// Resets mc_progress_counter() to 0 on entry; the k_mc_threads worker
// threads each increment it (relaxed) as their slice's runs complete.
MCStats run_monte_carlo(const MCConfig& cfg);

std::atomic<uint32_t>& mc_progress_counter();

// Wilson-Hilferty chi-squared quantile approximation (math.md §6).
double chi_squared_quantile(double p, double dof);

struct ConsistencyBounds {
    double lower = 0.0;
    double upper = 0.0;
};
ConsistencyBounds nees_bounds(size_t n_runs);
ConsistencyBounds nis_bounds(size_t n_runs);

}
