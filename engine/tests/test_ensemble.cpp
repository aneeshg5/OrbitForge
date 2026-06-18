#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>

#include "monte_carlo/ensemble.hpp"
#include "integrators/rk4.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::monte_carlo;
using namespace orbitforge::dynamics;

namespace {
constexpr double k_r0 = k_re + 408e3;  // ISS-like circular orbit radius
double iss_v0() { return std::sqrt(k_mu / k_r0); }
}  // namespace

TEST(Ensemble, SetGetRoundTrip) {
    EnsembleState<4> ens;
    Eigen::Matrix<double, 6, 1> x;
    x << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    ens.set(2, x);
    EXPECT_TRUE(ens.get(2).isApprox(x));
}

// One run inside an ensemble of size 1 must match an independent rk4_step
// call exactly — the ensemble machinery (get/set, SoA round-trip) must not
// perturb the numerics.
TEST(Ensemble, SingleRunMatchesDirectRk4Step) {
    PerturbationConfig cfg;
    cfg.enable_j2 = true;
    cfg.enable_drag = false;
    cfg.enable_srp = false;

    Eigen::Matrix<double, 6, 1> x0;
    x0 << k_r0, 0.0, 0.0, 0.0, iss_v0(), 0.0;

    EnsembleState<1> ens;
    ens.set(0, x0);
    step_ensemble(ens, 1, 10.0, k_j2000_jd, cfg);

    auto f = [&cfg](double, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = compute_acceleration(s.head<3>(), s.tail<3>(), k_j2000_jd, cfg);
        return ds;
    };
    const Eigen::Matrix<double, 6, 1> expected = rk4_step(x0, 0.0, 10.0, f);

    EXPECT_TRUE(ens.get(0).isApprox(expected, 1e-12));
}

// Runs must propagate independently — no cross-contamination between SoA
// lanes. Two runs start at different points on the same circular orbit;
// after one step each must have moved along its own trajectory only.
TEST(Ensemble, RunsAreIndependent) {
    PerturbationConfig cfg;
    cfg.enable_j2 = false;
    cfg.enable_drag = false;
    cfg.enable_srp = false;

    const double v0 = iss_v0();
    Eigen::Matrix<double, 6, 1> x0_a, x0_b;
    x0_a << k_r0, 0.0, 0.0, 0.0, v0, 0.0;
    x0_b << 0.0, k_r0, 0.0, -v0, 0.0, 0.0;  // 90 degrees ahead, same orbit

    EnsembleState<2> ens;
    ens.set(0, x0_a);
    ens.set(1, x0_b);
    step_ensemble(ens, 2, 10.0, k_j2000_jd, cfg);

    auto f = [&cfg](double, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = compute_acceleration(s.head<3>(), s.tail<3>(), k_j2000_jd, cfg);
        return ds;
    };
    const Eigen::Matrix<double, 6, 1> expected_a = rk4_step(x0_a, 0.0, 10.0, f);
    const Eigen::Matrix<double, 6, 1> expected_b = rk4_step(x0_b, 0.0, 10.0, f);

    EXPECT_TRUE(ens.get(0).isApprox(expected_a, 1e-12));
    EXPECT_TRUE(ens.get(1).isApprox(expected_b, 1e-12));
    // And the two results must differ from each other.
    EXPECT_FALSE(ens.get(0).isApprox(ens.get(1), 1.0));
}

// Kepler closure check (same criterion as test_rk4.cpp) run through the
// ensemble path for N=8 identical orbits, one full ISS period — confirms
// the SoA batch loop doesn't accumulate extra numerical error vs the
// already-validated single-run RK4 path.
TEST(Ensemble, KeplerClosureOverFullPeriod) {
    PerturbationConfig cfg;
    cfg.enable_j2 = false;
    cfg.enable_drag = false;
    cfg.enable_srp = false;

    const double v0 = iss_v0();
    Eigen::Matrix<double, 6, 1> x0;
    x0 << k_r0, 0.0, 0.0, 0.0, v0, 0.0;

    constexpr size_t n_runs = 8;
    auto ens = std::make_unique<EnsembleState<n_runs>>();
    for (size_t i = 0; i < n_runs; ++i) ens->set(i, x0);

    constexpr double dt = 10.0;
    const double period = 2.0 * M_PI * std::sqrt(k_r0 * k_r0 * k_r0 / k_mu);

    double t = 0.0;
    while (t + dt <= period) {
        step_ensemble(*ens, n_runs, dt, k_j2000_jd, cfg);
        t += dt;
    }
    if (period - t > 1e-10) {
        step_ensemble(*ens, n_runs, period - t, k_j2000_jd, cfg);
    }

    for (size_t i = 0; i < n_runs; ++i) {
        const Eigen::Vector3d final_pos = ens->get(i).head<3>();
        const double drift = (final_pos - x0.head<3>()).norm();
        EXPECT_LT(drift, 1.0) << "run " << i << " position drift after one period";
    }
}

// step_ensemble_fast() (batched gravity+J2 array kernel) must produce the
// same result as step_ensemble() (generic per-run Eigen path) to within
// floating-point reordering tolerance — it is a performance-motivated
// reformulation of the identical RK4(gravity+J2) math, not a different
// model.
TEST(Ensemble, FastPathMatchesGenericPathWithJ2) {
    PerturbationConfig cfg;
    cfg.enable_j2 = true;
    cfg.enable_drag = false;
    cfg.enable_srp = false;

    constexpr size_t n_runs = 5;
    const double v0 = iss_v0();

    EnsembleState<n_runs> ens_generic;
    EnsembleState<n_runs> ens_fast;
    for (size_t i = 0; i < n_runs; ++i) {
        Eigen::Matrix<double, 6, 1> x0;
        const double spread = static_cast<double>(i) * 1000.0;
        x0 << k_r0 + spread, spread * 0.5, 0.0, 0.0, v0, spread * 0.001;
        ens_generic.set(i, x0);
        ens_fast.set(i, x0);
    }

    step_ensemble(ens_generic, n_runs, 10.0, k_j2000_jd, cfg);

    EnsembleWorkspace<n_runs> ws;
    step_ensemble_fast(ens_fast, n_runs, 10.0, /*enable_j2=*/true, ws);

    for (size_t i = 0; i < n_runs; ++i) {
        EXPECT_TRUE(ens_fast.get(i).isApprox(ens_generic.get(i), 1e-9))
            << "run " << i << " fast vs generic mismatch";
    }
}

TEST(Ensemble, FastPathMatchesGenericPathTwoBodyOnly) {
    PerturbationConfig cfg;
    cfg.enable_j2 = false;
    cfg.enable_drag = false;
    cfg.enable_srp = false;

    constexpr size_t n_runs = 3;
    EnsembleState<n_runs> ens_generic;
    EnsembleState<n_runs> ens_fast;
    Eigen::Matrix<double, 6, 1> x0;
    x0 << k_r0, 0.0, 0.0, 0.0, iss_v0(), 0.0;
    for (size_t i = 0; i < n_runs; ++i) {
        ens_generic.set(i, x0);
        ens_fast.set(i, x0);
    }

    step_ensemble(ens_generic, n_runs, 10.0, k_j2000_jd, cfg);

    EnsembleWorkspace<n_runs> ws;
    step_ensemble_fast(ens_fast, n_runs, 10.0, /*enable_j2=*/false, ws);

    for (size_t i = 0; i < n_runs; ++i) {
        EXPECT_TRUE(ens_fast.get(i).isApprox(ens_generic.get(i), 1e-9));
    }
}
