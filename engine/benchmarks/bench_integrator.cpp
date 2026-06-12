#include <chrono>
#include <cstdio>
#include <Eigen/Dense>

#include "dynamics/eom.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::dynamics;
using Clock = std::chrono::high_resolution_clock;

static double bench_gravity_step(int iterations) {
    const Eigen::Vector3d pos{k_re + 408e3, 0.0, 0.0};
    const Eigen::Vector3d vel{0.0, 7700.0, 0.0};

    PerturbationConfig cfg;
    cfg.enable_j2   = true;
    cfg.enable_drag = true;
    cfg.enable_srp  = false;

    auto t0 = Clock::now();
    Eigen::Vector3d accel;
    for (int i = 0; i < iterations; ++i) {
        accel = compute_acceleration(pos, vel, 2451545.0, cfg);
    }
    auto t1 = Clock::now();

    // Use accel to prevent optimization
    volatile double sink = accel.norm();
    (void)sink;

    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
}

int main() {
    constexpr int N = 1'000'000;
    const double us = bench_gravity_step(N);
    std::printf("compute_acceleration (J2+drag, %d iterations): %.3f μs/call\n", N, us);
    return 0;
}
