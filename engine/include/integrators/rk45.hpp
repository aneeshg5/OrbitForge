#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Dense>

namespace orbitforge {

namespace detail {
// Dormand-Prince RK45 Butcher tableau — Hairer & Wanner "Solving ODEs I", Table 5.2
constexpr double k_dp_a21 =     1.0/5.0;
constexpr double k_dp_a31 =     3.0/40.0;
constexpr double k_dp_a32 =     9.0/40.0;
constexpr double k_dp_a41 =    44.0/45.0;
constexpr double k_dp_a42 =   -56.0/15.0;
constexpr double k_dp_a43 =    32.0/9.0;
constexpr double k_dp_a51 = 19372.0/6561.0;
constexpr double k_dp_a52 = -25360.0/2187.0;
constexpr double k_dp_a53 =  64448.0/6561.0;
constexpr double k_dp_a54 =   -212.0/729.0;
constexpr double k_dp_a61 =  9017.0/3168.0;
constexpr double k_dp_a62 =  -355.0/33.0;
constexpr double k_dp_a63 = 46732.0/5247.0;
constexpr double k_dp_a64 =    49.0/176.0;
constexpr double k_dp_a65 = -5103.0/18656.0;

// 5th-order output weights
constexpr double k_dp_b1 =    35.0/384.0;
constexpr double k_dp_b3 =   500.0/1113.0;
constexpr double k_dp_b4 =   125.0/192.0;
constexpr double k_dp_b5 = -2187.0/6784.0;
constexpr double k_dp_b6 =    11.0/84.0;

// Error coefficients: e = b_5th − b_4th (math.md §2.2)
constexpr double k_dp_e1 =    71.0/57600.0;
constexpr double k_dp_e3 =   -71.0/16695.0;
constexpr double k_dp_e4 =    71.0/1920.0;
constexpr double k_dp_e5 = -17253.0/339200.0;
constexpr double k_dp_e6 =    22.0/525.0;
constexpr double k_dp_e7 =    -1.0/40.0;
} // namespace detail

// Attempt one Dormand-Prince RK45 step. Returns {x_5th, err_norm}.
// err_norm < 1 → step accepted; err_norm >= 1 → rejected, retry with smaller h.
// State: Eigen fixed-size vector. Dynamics: (double t, const State&) -> State.
template <typename State, typename Dynamics>
std::pair<State, double> rk45_try_step(
    const State& x, double t, double h, Dynamics&& f, double atol, double rtol)
{
    using namespace detail;

    const State k1 = f(t,              x);
    const State k2 = f(t + h/5.0,     x + h * k_dp_a21*k1);
    const State k3 = f(t + 3.0*h/10.0,x + h * (k_dp_a31*k1 + k_dp_a32*k2));
    const State k4 = f(t + 4.0*h/5.0, x + h * (k_dp_a41*k1 + k_dp_a42*k2 + k_dp_a43*k3));
    const State k5 = f(t + 8.0*h/9.0, x + h * (k_dp_a51*k1 + k_dp_a52*k2 + k_dp_a53*k3 + k_dp_a54*k4));
    const State k6 = f(t + h,          x + h * (k_dp_a61*k1 + k_dp_a62*k2 + k_dp_a63*k3 + k_dp_a64*k4 + k_dp_a65*k5));

    const State x5 = x + h * (k_dp_b1*k1 + k_dp_b3*k3 + k_dp_b4*k4 + k_dp_b5*k5 + k_dp_b6*k6);
    const State k7 = f(t + h, x5);

    const State err = h * (k_dp_e1*k1 + k_dp_e3*k3 + k_dp_e4*k4 + k_dp_e5*k5 + k_dp_e6*k6 + k_dp_e7*k7);

    // Scaled infinity-norm: step accepted when err_norm <= 1
    const double tol      = atol + rtol * x.template lpNorm<Eigen::Infinity>();
    const double err_norm = err.template lpNorm<Eigen::Infinity>() / tol;

    return {x5, err_norm};
}

// Integrate from t0 to t1 using adaptive Dormand-Prince RK45 (math.md §2.2).
// Defaults match CLAUDE.md §6: atol=1e-6 m, rtol=1e-9.
template <typename State, typename Dynamics>
State rk45_integrate(
    const State& x0, double t0, double t1, Dynamics&& f,
    double atol = 1e-6, double rtol = 1e-9, double h_init = 10.0)
{
    constexpr double k_safety     = 0.9;
    constexpr double k_min_factor = 0.1;
    constexpr double k_max_factor = 5.0;

    State  x = x0;
    double t = t0;
    double h = std::min(h_init, t1 - t0);

    while (t < t1) {
        if (t + h > t1) h = t1 - t;

        auto [x_new, err_norm] = rk45_try_step(x, t, h, f, atol, rtol);

        if (err_norm <= 1.0) {
            t += h;
            x  = std::move(x_new);
        }

        // h_new = h * clamp(0.9 * (1/err_norm)^0.2, 0.1, 5) — math.md §2.2
        const double factor = std::clamp(
            k_safety * std::pow(1.0 / std::max(err_norm, 1e-10), 0.2),
            k_min_factor, k_max_factor);
        h *= factor;
    }

    return x;
}

} // namespace orbitforge
