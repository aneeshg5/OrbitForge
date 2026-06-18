#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Dense>

#include "constants.hpp"
#include "dynamics/eom.hpp"
#include "integrators/rk4.hpp"

namespace orbitforge::monte_carlo {

// Maximum concurrent Monte Carlo runs (UI exposes 100-5000).
constexpr size_t k_mc_max_runs = 5000;

// Structure-of-Arrays state buffer for N independent true trajectories.
// Each of the 6 state components is a contiguous array
// spanning all runs, so step_ensemble()'s inner loop reads/writes 6
// sequential streams per iteration instead of striding through an
// interleaved per-run struct that also carries a covariance block the loop
// never touches — see bench_monte_carlo.cpp for the measured throughput
// delta against that AoS layout.
template <size_t MaxN>
struct EnsembleState {
    std::array<double, MaxN> pos_x{};
    std::array<double, MaxN> pos_y{};
    std::array<double, MaxN> pos_z{};
    std::array<double, MaxN> vel_x{};
    std::array<double, MaxN> vel_y{};
    std::array<double, MaxN> vel_z{};

    void set(size_t i, const Eigen::Matrix<double, 6, 1>& x) noexcept {
        pos_x[i] = x(0); pos_y[i] = x(1); pos_z[i] = x(2);
        vel_x[i] = x(3); vel_y[i] = x(4); vel_z[i] = x(5);
    }

    Eigen::Matrix<double, 6, 1> get(size_t i) const noexcept {
        Eigen::Matrix<double, 6, 1> x;
        x << pos_x[i], pos_y[i], pos_z[i], vel_x[i], vel_y[i], vel_z[i];
        return x;
    }
};

// Propagates runs [0, n) by one RK4 step of dt seconds, all sharing the
// same dynamics config and Julian date. Per-run randomness (initial
// condition spread, process noise injection) is applied by the caller
// before/after this call — this function is only the batch-propagation
// step, which is the part that benefits from the SoA layout.
template <size_t MaxN>
void step_ensemble(EnsembleState<MaxN>& ens, size_t n, double dt, double julian_date,
                    const dynamics::PerturbationConfig& cfg) noexcept {
    auto f = [&cfg, julian_date](double, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), julian_date, cfg);
        return ds;
    };
    for (size_t i = 0; i < n; ++i) {
        ens.set(i, rk4_step(ens.get(i), 0.0, dt, f));
    }
}

// Batched two-body + J2 gravity acceleration (math.md §1.1-1.2), evaluated
// directly on contiguous double[] streams rather than through the generic
// per-run compute_acceleration() Vector3d API. This is what actually lets
// step_ensemble_fast() vectorize: every loop body below touches only
// sequential arrays and has no data-dependent branch inside the loop (the
// enable_j2 toggle selects which loop runs, not an inner-loop branch), so
// the compiler can pack iterations into SIMD lanes.
//
// Deliberately scoped to gravity+J2 only — the dominant LEO perturbation
// and the same scope as the EKF/UKF analytical Jacobian. Drag depends on a
// branchy 7-band atmosphere lookup and SRP depends on a shared
// per-tick (not per-run) sun-direction vector; neither benefits from this
// treatment, so step_ensemble() above remains the correct, general path
// whenever either is enabled.
template <size_t MaxN>
void accel_gravity_j2_batch(const std::array<double, MaxN>& px,
                             const std::array<double, MaxN>& py,
                             const std::array<double, MaxN>& pz,
                             size_t n, bool enable_j2,
                             std::array<double, MaxN>& ax,
                             std::array<double, MaxN>& ay,
                             std::array<double, MaxN>& az) noexcept {
    if (enable_j2) {
        for (size_t i = 0; i < n; ++i) {
            const double x = px[i], y = py[i], z = pz[i];
            const double r2 = x * x + y * y + z * z;
            const double r  = std::sqrt(r2);
            const double g  = -k_mu / (r2 * r);
            const double factor = 1.5 * k_j2 * k_mu * (k_re * k_re) / (r2 * r2 * r);
            const double zr2 = (z / r) * (z / r);
            ax[i] = g * x + factor * x * (5.0 * zr2 - 1.0);
            ay[i] = g * y + factor * y * (5.0 * zr2 - 1.0);
            az[i] = g * z + factor * z * (5.0 * zr2 - 3.0);
        }
    } else {
        for (size_t i = 0; i < n; ++i) {
            const double x = px[i], y = py[i], z = pz[i];
            const double r2 = x * x + y * y + z * z;
            const double r  = std::sqrt(r2);
            const double g  = -k_mu / (r2 * r);
            ax[i] = g * x;
            ay[i] = g * y;
            az[i] = g * z;
        }
    }
}

// Scratch buffers for step_ensemble_fast(), sized for the largest supported
// ensemble. Allocate once (e.g. at Monte Carlo run start, via
// std::make_unique) and reuse across ticks — never inside the per-tick hot
// loop itself.
template <size_t MaxN>
struct EnsembleWorkspace {
    std::array<double, MaxN> p1x{}, p1y{}, p1z{}, v1x{}, v1y{}, v1z{};
    std::array<double, MaxN> p2x{}, p2y{}, p2z{}, v2x{}, v2y{}, v2z{};
    std::array<double, MaxN> p3x{}, p3y{}, p3z{}, v3x{}, v3y{}, v3z{};
    std::array<double, MaxN> a0x{}, a0y{}, a0z{};
    std::array<double, MaxN> a1x{}, a1y{}, a1z{};
    std::array<double, MaxN> a2x{}, a2y{}, a2z{};
    std::array<double, MaxN> a3x{}, a3y{}, a3z{};
};

// Batched RK4 step using accel_gravity_j2_batch(), applying the standard
// split-RK4 form for second-order systems ẍ = a(x):
//   a0 = a(p);                 p1 = p + h/2·v;   v1 = v + h/2·a0
//   a1 = a(p1);                p2 = p + h/2·v1;  v2 = v + h/2·a1
//   a2 = a(p2);                p3 = p + h·v2;    v3 = v + h·a2
//   a3 = a(p3)
//   p_new = p + h/6·(v + 2v1 + 2v2 + v3)
//   v_new = v + h/6·(a0 + 2a1 + 2a2 + a3)
// Algebraically identical to applying rk4_step() to the generic dynamics
// f(p,v) = (v, a(p)) — verified against it in test_ensemble.cpp — but
// expressed as plain array arithmetic instead of per-run Eigen::Matrix
// construction, which is what makes it vectorizable.
template <size_t MaxN>
void step_ensemble_fast(EnsembleState<MaxN>& ens, size_t n, double dt, bool enable_j2,
                         EnsembleWorkspace<MaxN>& ws) noexcept {
    accel_gravity_j2_batch(ens.pos_x, ens.pos_y, ens.pos_z, n, enable_j2, ws.a0x, ws.a0y, ws.a0z);
    for (size_t i = 0; i < n; ++i) {
        ws.p1x[i] = ens.pos_x[i] + 0.5 * dt * ens.vel_x[i];
        ws.p1y[i] = ens.pos_y[i] + 0.5 * dt * ens.vel_y[i];
        ws.p1z[i] = ens.pos_z[i] + 0.5 * dt * ens.vel_z[i];
        ws.v1x[i] = ens.vel_x[i] + 0.5 * dt * ws.a0x[i];
        ws.v1y[i] = ens.vel_y[i] + 0.5 * dt * ws.a0y[i];
        ws.v1z[i] = ens.vel_z[i] + 0.5 * dt * ws.a0z[i];
    }

    accel_gravity_j2_batch(ws.p1x, ws.p1y, ws.p1z, n, enable_j2, ws.a1x, ws.a1y, ws.a1z);
    for (size_t i = 0; i < n; ++i) {
        ws.p2x[i] = ens.pos_x[i] + 0.5 * dt * ws.v1x[i];
        ws.p2y[i] = ens.pos_y[i] + 0.5 * dt * ws.v1y[i];
        ws.p2z[i] = ens.pos_z[i] + 0.5 * dt * ws.v1z[i];
        ws.v2x[i] = ens.vel_x[i] + 0.5 * dt * ws.a1x[i];
        ws.v2y[i] = ens.vel_y[i] + 0.5 * dt * ws.a1y[i];
        ws.v2z[i] = ens.vel_z[i] + 0.5 * dt * ws.a1z[i];
    }

    accel_gravity_j2_batch(ws.p2x, ws.p2y, ws.p2z, n, enable_j2, ws.a2x, ws.a2y, ws.a2z);
    for (size_t i = 0; i < n; ++i) {
        ws.p3x[i] = ens.pos_x[i] + dt * ws.v2x[i];
        ws.p3y[i] = ens.pos_y[i] + dt * ws.v2y[i];
        ws.p3z[i] = ens.pos_z[i] + dt * ws.v2z[i];
        ws.v3x[i] = ens.vel_x[i] + dt * ws.a2x[i];
        ws.v3y[i] = ens.vel_y[i] + dt * ws.a2y[i];
        ws.v3z[i] = ens.vel_z[i] + dt * ws.a2z[i];
    }

    accel_gravity_j2_batch(ws.p3x, ws.p3y, ws.p3z, n, enable_j2, ws.a3x, ws.a3y, ws.a3z);
    constexpr double k_sixth = 1.0 / 6.0;
    for (size_t i = 0; i < n; ++i) {
        ens.pos_x[i] += dt * k_sixth * (ens.vel_x[i] + 2.0 * ws.v1x[i] + 2.0 * ws.v2x[i] + ws.v3x[i]);
        ens.pos_y[i] += dt * k_sixth * (ens.vel_y[i] + 2.0 * ws.v1y[i] + 2.0 * ws.v2y[i] + ws.v3y[i]);
        ens.pos_z[i] += dt * k_sixth * (ens.vel_z[i] + 2.0 * ws.v1z[i] + 2.0 * ws.v2z[i] + ws.v3z[i]);
        ens.vel_x[i] += dt * k_sixth * (ws.a0x[i] + 2.0 * ws.a1x[i] + 2.0 * ws.a2x[i] + ws.a3x[i]);
        ens.vel_y[i] += dt * k_sixth * (ws.a0y[i] + 2.0 * ws.a1y[i] + 2.0 * ws.a2y[i] + ws.a3y[i]);
        ens.vel_z[i] += dt * k_sixth * (ws.a0z[i] + 2.0 * ws.a1z[i] + 2.0 * ws.a2z[i] + ws.a3z[i]);
    }
}

}  // namespace orbitforge::monte_carlo
