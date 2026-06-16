#pragma once

namespace orbitforge {

// Fixed-step RK4 integrator — math.md §2.1
// State: Eigen fixed-size vector (operator+ and scalar operator* required).
// Dynamics: callable (double t, const State&) -> State  [returns ẋ = f(t, x)]
template <typename State, typename Dynamics>
State rk4_step(const State& x, double t, double h, Dynamics&& f) {
    const State k1 = f(t,         x);
    const State k2 = f(t + h*0.5, x + (h * 0.5) * k1);
    const State k3 = f(t + h*0.5, x + (h * 0.5) * k2);
    const State k4 = f(t + h,     x + h          * k3);
    return x + (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

} // namespace orbitforge
