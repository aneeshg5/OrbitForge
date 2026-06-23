#include <chrono>
#include <cstdio>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "monte_carlo/ensemble.hpp"
#include "monte_carlo/mc_runner.hpp"
#include "dynamics/eom.hpp"
#include "integrators/rk4.hpp"
#include "constants.hpp"

namespace orbitforge::benchmarks {

using Clock = std::chrono::high_resolution_clock;
using orbitforge::dynamics::PerturbationConfig;
using orbitforge::dynamics::compute_acceleration;
using orbitforge::monte_carlo::EnsembleState;
using orbitforge::monte_carlo::step_ensemble;

namespace {

Eigen::Matrix<double, 6, 1> iss_state() {
    const double r0 = orbitforge::k_re + 408e3;
    const double v0 = std::sqrt(orbitforge::k_mu / r0);
    Eigen::Matrix<double, 6, 1> x;
    x << r0, 0.0, 0.0, 0.0, v0, 0.0;
    return x;
}

struct RunStateAoS {
    double px, py, pz, vx, vy, vz;
    double cov[36];
};

using orbitforge::monte_carlo::EnsembleWorkspace;
using orbitforge::monte_carlo::step_ensemble_fast;

double bench_soa_generic_throughput(size_t n_runs, int n_steps) {
    auto ens = std::make_unique<EnsembleState<orbitforge::monte_carlo::k_mc_max_runs>>();
    const Eigen::Matrix<double, 6, 1> x0 = iss_state();
    for (size_t i = 0; i < n_runs; ++i) ens->set(i, x0);

    PerturbationConfig cfg;
    cfg.enable_j2 = true;
    cfg.enable_drag = false;
    cfg.enable_srp = false;

    const auto t0 = Clock::now();
    for (int step = 0; step < n_steps; ++step) {
        step_ensemble(*ens, n_runs, 10.0, k_j2000_jd, cfg);
    }
    const auto t1 = Clock::now();

    volatile double sink = ens->pos_x[0] + ens->pos_x[n_runs - 1];
    (void)sink;

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    return static_cast<double>(n_runs) * n_steps / seconds;
}

double bench_soa_fast_throughput(size_t n_runs, int n_steps) {
    using k_max = std::integral_constant<size_t, orbitforge::monte_carlo::k_mc_max_runs>;
    auto ens = std::make_unique<EnsembleState<k_max::value>>();
    auto ws  = std::make_unique<EnsembleWorkspace<k_max::value>>();
    const Eigen::Matrix<double, 6, 1> x0 = iss_state();
    for (size_t i = 0; i < n_runs; ++i) ens->set(i, x0);

    const auto t0 = Clock::now();
    for (int step = 0; step < n_steps; ++step) {
        step_ensemble_fast(*ens, n_runs, 10.0, true, *ws);
    }
    const auto t1 = Clock::now();

    volatile double sink = ens->pos_x[0] + ens->pos_x[n_runs - 1];
    (void)sink;

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    return static_cast<double>(n_runs) * n_steps / seconds;
}

double bench_aos_throughput(size_t n_runs, int n_steps) {
    std::vector<RunStateAoS> runs(n_runs);
    const Eigen::Matrix<double, 6, 1> x0 = iss_state();
    for (auto& r : runs) {
        r.px = x0(0); r.py = x0(1); r.pz = x0(2);
        r.vx = x0(3); r.vy = x0(4); r.vz = x0(5);
    }

    PerturbationConfig cfg;
    cfg.enable_j2 = true;
    cfg.enable_drag = false;
    cfg.enable_srp = false;

    auto f = [&cfg](double, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = compute_acceleration(s.head<3>(), s.tail<3>(), k_j2000_jd, cfg);
        return ds;
    };

    const auto t0 = Clock::now();
    for (int step = 0; step < n_steps; ++step) {
        for (auto& r : runs) {
            Eigen::Matrix<double, 6, 1> x;
            x << r.px, r.py, r.pz, r.vx, r.vy, r.vz;
            x = rk4_step(x, 0.0, 10.0, f);
            r.px = x(0); r.py = x(1); r.pz = x(2);
            r.vx = x(3); r.vy = x(4); r.vz = x(5);
        }
    }
    const auto t1 = Clock::now();

    volatile double sink = runs.front().px + runs.back().px;
    (void)sink;

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    return static_cast<double>(n_runs) * n_steps / seconds;
}

double bench_mc_runner_wall_ms(size_t n_runs, int n_steps) {
    orbitforge::monte_carlo::MCConfig cfg;
    cfg.n_runs = n_runs;
    cfg.n_steps = n_steps;
    cfg.dt = 10.0;
    cfg.x0 = iss_state();
    cfg.filter = orbitforge::monte_carlo::FilterKind::ekf;
    cfg.gps_sigma = 10.0;
    cfg.q_pos = 1.0;
    cfg.q_vel = 0.01;

    const auto t0 = Clock::now();
    const auto stats = orbitforge::monte_carlo::run_monte_carlo(cfg);
    const auto t1 = Clock::now();

    volatile double sink = stats.nees.empty() ? 0.0 : stats.nees.front();
    (void)sink;

    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

}

void run_monte_carlo_benchmarks() {
    constexpr size_t n_runs = 1000;
    constexpr int n_steps = 100;

    const double aos_throughput        = bench_aos_throughput(n_runs, n_steps);
    const double soa_generic_throughput = bench_soa_generic_throughput(n_runs, n_steps);
    const double soa_fast_throughput    = bench_soa_fast_throughput(n_runs, n_steps);

    std::printf("Monte Carlo ensemble RK4 batch (N=%zu runs, %d steps):\n", n_runs, n_steps);
    std::printf("  AoS, scalar per-run:                 %.3e ensemble-steps/sec\n", aos_throughput);
    std::printf("  SoA, scalar per-run (step_ensemble):  %.3e ensemble-steps/sec  (%.1f%% vs AoS)\n",
                soa_generic_throughput, (soa_generic_throughput / aos_throughput - 1.0) * 100.0);
    std::printf("  SoA, batched kernel (step_ensemble_fast): %.3e ensemble-steps/sec  (%.1f%% vs AoS)\n",
                soa_fast_throughput, (soa_fast_throughput / aos_throughput - 1.0) * 100.0);

    const double mc_ms = bench_mc_runner_wall_ms(1000, 1000);
    std::printf("mc_runner full campaign (N=1000 runs x 1000 steps, EKF, %zu threads): %.1f ms\n",
                orbitforge::monte_carlo::k_mc_threads, mc_ms);
}

}
