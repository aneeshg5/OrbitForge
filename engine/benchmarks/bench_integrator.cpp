#include <chrono>
#include <cstdio>
#include <Eigen/Dense>

#include "dynamics/eom.hpp"
#include "integrators/rk4.hpp"
#include "constants.hpp"
#include "memory/ring_buffer.hpp"

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

    volatile double sink = accel.norm();
    (void)sink;

    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
}

static double bench_rk4_step(int iterations) {
    PerturbationConfig cfg;
    cfg.enable_j2   = true;
    cfg.enable_drag = true;
    cfg.enable_srp  = false;

    Eigen::Matrix<double, 6, 1> x;
    x << k_re + 408e3, 0.0, 0.0, 0.0, 7700.0, 0.0;

    auto f = [&cfg](double, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = compute_acceleration(s.head<3>(), s.tail<3>(), 2451545.0, cfg);
        return ds;
    };

    const auto t0 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        x = rk4_step(x, 0.0, 10.0, f);
    }
    const auto t1 = Clock::now();

    volatile double sink = x.norm();
    (void)sink;

    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
}

static double bench_ring_buffer_throughput(int iterations) {
    orbitforge::memory::SPSCRingBuffer<int, 1024> rb;
    int sink = 0;

    const auto t0 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        rb.push(i);
        rb.pop(sink);
    }
    const auto t1 = Clock::now();

    volatile int vsink = sink;
    (void)vsink;

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    return iterations / seconds;
}

namespace orbitforge::benchmarks {
void run_filter_benchmarks();
void run_monte_carlo_benchmarks();
}

int main() {
    constexpr int N = 1'000'000;
    const double accel_us = bench_gravity_step(N);
    std::printf("compute_acceleration (J2+drag, %d iterations):  %.3f us/call\n", N, accel_us);

    const double rk4_us = bench_rk4_step(N);
    std::printf("rk4_step (6-state, J2+drag, %d iterations):     %.3f us/step\n", N, rk4_us);

    orbitforge::benchmarks::run_filter_benchmarks();

    const double ring_buffer_fps = bench_ring_buffer_throughput(10'000'000);
    std::printf("ring buffer push+pop pairs/sec (single-thread, %d iterations): %.2e /sec\n",
                10'000'000, ring_buffer_fps);

    orbitforge::benchmarks::run_monte_carlo_benchmarks();
    return 0;
}
