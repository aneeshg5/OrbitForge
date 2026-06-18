#include "monte_carlo/mc_runner.hpp"

#include <cmath>
#include <random>
#include <thread>
#include <utility>
#include <vector>

#include "constants.hpp"
#include "dynamics/eom.hpp"
#include "filters/ekf.hpp"
#include "filters/kf.hpp"
#include "filters/ukf.hpp"
#include "integrators/rk4.hpp"

namespace orbitforge::monte_carlo {

namespace {

double std_normal_cdf(double z) {
    return 0.5 * std::erfc(-z / std::sqrt(2.0));
}

// Bisection on the exact normal CDF — no transcribed rational-approximation
// coefficients to get wrong, and this runs once per chi_squared_quantile()
// call (campaign-summary time, not per-tick), so the extra iterations cost
// nothing that matters.
double std_normal_quantile(double p) {
    double lo = -10.0, hi = 10.0;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (std_normal_cdf(mid) < p) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

// Per-thread accumulators for one slice of runs. Summed once across threads
// after all threads join — no shared mutable state during the parallel
// phase, so no synchronization is needed inside run_one().
struct PartialSums {
    std::vector<double> sq_pos_err;
    std::vector<double> sq_vel_err;
    std::vector<double> nees_sum;
    std::vector<double> nis_sum;
    std::vector<int>    nees_cnt;
    std::vector<int>    nis_cnt;
    std::vector<double> final_pos_err;  // size = this thread's run-slice count

    PartialSums(int n_steps, size_t slice_count)
        : sq_pos_err(static_cast<size_t>(n_steps), 0.0),
          sq_vel_err(static_cast<size_t>(n_steps), 0.0),
          nees_sum(static_cast<size_t>(n_steps), 0.0),
          nis_sum(static_cast<size_t>(n_steps), 0.0),
          nees_cnt(static_cast<size_t>(n_steps), 0),
          nis_cnt(static_cast<size_t>(n_steps), 0),
          final_pos_err(slice_count, 0.0) {}
};

template <typename Filter>
Filter make_filter(const MCConfig& cfg) {
    Filter f;
    f.x = cfg.x0;
    f.P.setZero();
    f.P.diagonal().template head<3>().setConstant(cfg.p0_pos * cfg.p0_pos);
    f.P.diagonal().template tail<3>().setConstant(cfg.p0_vel * cfg.p0_vel);
    f.Q.setZero();
    f.Q.diagonal().template head<3>().setConstant(cfg.q_pos * cfg.q_pos);
    f.Q.diagonal().template tail<3>().setConstant(cfg.q_vel * cfg.q_vel);
    f.R = Eigen::Matrix3d::Identity() * (cfg.gps_sigma * cfg.gps_sigma);
    return f;
}

// KF has no RK4/perturbation-model fields — no-op overload.
void configure_dynamics(filters::KalmanFilter&, const dynamics::PerturbationConfig&, double) {}

void configure_dynamics(filters::ExtendedKalmanFilter& f, const dynamics::PerturbationConfig& cfg,
                         double jd) {
    f.perturb_cfg = cfg;
    f.julian_date = jd;
}

void configure_dynamics(filters::UnscentedKalmanFilter& f, const dynamics::PerturbationConfig& cfg,
                         double jd) {
    f.perturb_cfg = cfg;
    f.julian_date = jd;
    f.S = f.P.llt().matrixL();
}

template <typename Filter>
void run_one(const MCConfig& cfg, unsigned run_seed, PartialSums& sums, double& final_pos_err_out) {
    std::mt19937 rng(run_seed);
    std::normal_distribution<double> ic_pos(0.0, cfg.p0_pos);
    std::normal_distribution<double> ic_vel(0.0, cfg.p0_vel);
    std::normal_distribution<double> gps_noise(0.0, cfg.gps_sigma);
    std::normal_distribution<double> proc_pos(0.0, cfg.q_pos);
    std::normal_distribution<double> proc_vel(0.0, cfg.q_vel);

    dynamics::PerturbationConfig two_body;
    two_body.enable_j2 = false;
    two_body.enable_drag = false;
    two_body.enable_srp = false;

    auto f_dyn = [&two_body](double, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), k_j2000_jd, two_body);
        return ds;
    };

    Eigen::Matrix<double, 6, 1> x_true = cfg.x0;

    Filter flt = make_filter<Filter>(cfg);
    configure_dynamics(flt, two_body, k_j2000_jd);
    flt.x(0) += ic_pos(rng); flt.x(1) += ic_pos(rng); flt.x(2) += ic_pos(rng);
    flt.x(3) += ic_vel(rng); flt.x(4) += ic_vel(rng); flt.x(5) += ic_vel(rng);

    Eigen::Matrix<double, 3, 6> H;
    H << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();

    for (int step = 0; step < cfg.n_steps; ++step) {
        x_true = rk4_step(x_true, 0.0, cfg.dt, f_dyn);
        x_true(0) += proc_pos(rng); x_true(1) += proc_pos(rng); x_true(2) += proc_pos(rng);
        x_true(3) += proc_vel(rng); x_true(4) += proc_vel(rng); x_true(5) += proc_vel(rng);

        flt.predict(cfg.dt);

        const Eigen::Vector3d z = x_true.head<3>() +
            Eigen::Vector3d(gps_noise(rng), gps_noise(rng), gps_noise(rng));

        // NIS computed pre-update, using the prior (predicted) covariance —
        // same convention as wasm_api.cpp's Simulation::step().
        const Eigen::Vector3d nu = z - H * flt.x;
        const Eigen::Matrix3d S = H * flt.P * H.transpose() + flt.R;
        sums.nis_sum[static_cast<size_t>(step)] += nu.dot(S.ldlt().solve(nu));
        sums.nis_cnt[static_cast<size_t>(step)]++;

        flt.update(z);

        const Eigen::Matrix<double, 6, 1> err = x_true - flt.x;
        const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(flt.P);
        if (llt.info() == Eigen::Success) {
            sums.nees_sum[static_cast<size_t>(step)] += err.dot(llt.solve(err));
            sums.nees_cnt[static_cast<size_t>(step)]++;
        }
        sums.sq_pos_err[static_cast<size_t>(step)] += err.head<3>().squaredNorm();
        sums.sq_vel_err[static_cast<size_t>(step)] += err.tail<3>().squaredNorm();

        if (step == cfg.n_steps - 1) final_pos_err_out = err.head<3>().norm();
    }
}

void run_slice(const MCConfig& cfg, size_t begin, size_t end, PartialSums& sums) {
    for (size_t run = begin; run < end; ++run) {
        const unsigned run_seed = cfg.seed + static_cast<unsigned>(run) * 7919u + 1u;
        double final_pos_err = 0.0;
        switch (cfg.filter) {
            case FilterKind::kf:  run_one<filters::KalmanFilter>(cfg, run_seed, sums, final_pos_err); break;
            case FilterKind::ekf: run_one<filters::ExtendedKalmanFilter>(cfg, run_seed, sums, final_pos_err); break;
            case FilterKind::ukf: run_one<filters::UnscentedKalmanFilter>(cfg, run_seed, sums, final_pos_err); break;
        }
        sums.final_pos_err[run - begin] = final_pos_err;
    }
}

}  // namespace

double chi_squared_quantile(double p, double dof) {
    const double z = std_normal_quantile(p);
    const double term = 1.0 - 2.0 / (9.0 * dof) + z * std::sqrt(2.0 / (9.0 * dof));
    return dof * term * term * term;
}

MCStats run_monte_carlo(const MCConfig& cfg) {
    const int n_steps = cfg.n_steps;
    const size_t n    = cfg.n_runs;
    const size_t base = n / k_mc_threads;
    const size_t rem  = n % k_mc_threads;

    std::vector<std::pair<size_t, size_t>> slices;  // (start, end) per thread, for the merge below
    slices.reserve(k_mc_threads);
    {
        size_t start = 0;
        for (size_t t = 0; t < k_mc_threads; ++t) {
            const size_t count = base + (t < rem ? 1 : 0);
            slices.emplace_back(start, start + count);
            start += count;
        }
    }

    std::vector<PartialSums> thread_sums;
    thread_sums.reserve(k_mc_threads);
    for (const auto& [s, e] : slices) thread_sums.emplace_back(n_steps, e - s);

    std::vector<std::thread> threads;
    threads.reserve(k_mc_threads);
    for (size_t t = 0; t < k_mc_threads; ++t) {
        threads.emplace_back(run_slice, std::cref(cfg), slices[t].first, slices[t].second, std::ref(thread_sums[t]));
    }
    for (auto& th : threads) th.join();

    MCStats stats;
    stats.rms_pos.assign(static_cast<size_t>(n_steps), 0.0);
    stats.rms_vel.assign(static_cast<size_t>(n_steps), 0.0);
    stats.nees.assign(static_cast<size_t>(n_steps), 0.0);
    stats.nis.assign(static_cast<size_t>(n_steps), 0.0);
    stats.final_pos_err.assign(n, 0.0);
    for (size_t t = 0; t < k_mc_threads; ++t) {
        const auto& [s, e] = slices[t];
        for (size_t i = 0; i < e - s; ++i) stats.final_pos_err[s + i] = thread_sums[t].final_pos_err[i];
    }

    for (int step = 0; step < n_steps; ++step) {
        const size_t s = static_cast<size_t>(step);
        double sq_pos = 0.0, sq_vel = 0.0, nees_sum = 0.0, nis_sum = 0.0;
        int nees_cnt = 0, nis_cnt = 0;
        for (const auto& ps : thread_sums) {
            sq_pos   += ps.sq_pos_err[s];
            sq_vel   += ps.sq_vel_err[s];
            nees_sum += ps.nees_sum[s];
            nis_sum  += ps.nis_sum[s];
            nees_cnt += ps.nees_cnt[s];
            nis_cnt  += ps.nis_cnt[s];
        }
        stats.rms_pos[s] = (n > 0) ? std::sqrt(sq_pos / static_cast<double>(n)) : 0.0;
        stats.rms_vel[s] = (n > 0) ? std::sqrt(sq_vel / static_cast<double>(n)) : 0.0;
        stats.nees[s] = (nees_cnt > 0) ? nees_sum / nees_cnt : 0.0;
        stats.nis[s]  = (nis_cnt > 0)  ? nis_sum / nis_cnt  : 0.0;
    }
    return stats;
}

ConsistencyBounds nees_bounds(size_t n_runs) {
    const double dof = 6.0 * static_cast<double>(n_runs);
    return {chi_squared_quantile(0.025, dof) / static_cast<double>(n_runs),
            chi_squared_quantile(0.975, dof) / static_cast<double>(n_runs)};
}

ConsistencyBounds nis_bounds(size_t n_runs) {
    const double dof = 3.0 * static_cast<double>(n_runs);
    return {chi_squared_quantile(0.025, dof) / static_cast<double>(n_runs),
            chi_squared_quantile(0.975, dof) / static_cast<double>(n_runs)};
}

}  // namespace orbitforge::monte_carlo
