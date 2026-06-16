#include <chrono>
#include <cstdio>
#include <cmath>
#include <Eigen/Dense>

#include "filters/kf.hpp"
#include "filters/ekf.hpp"
#include "filters/ukf.hpp"
#include "constants.hpp"

namespace orbitforge::benchmarks {

using Clock = std::chrono::high_resolution_clock;
using orbitforge::filters::KalmanFilter;
using orbitforge::filters::ExtendedKalmanFilter;
using orbitforge::filters::UnscentedKalmanFilter;

namespace {

constexpr double k_dt = 10.0;  // seconds, matches real-time sim tick

Eigen::Matrix<double, 6, 1> iss_state() {
    const double r0 = orbitforge::k_re + 408e3;
    const double v0 = std::sqrt(orbitforge::k_mu / r0);
    Eigen::Matrix<double, 6, 1> x;
    x << r0, 0.0, 0.0, 0.0, v0, 0.0;
    return x;
}

void configure(KalmanFilter& kf) {
    kf.x = iss_state();
    kf.P.setIdentity();
    kf.P *= 100.0;
    kf.Q.setZero();
    for (int i = 0; i < 3; ++i) kf.Q(i, i) = 1e-4;
    for (int i = 3; i < 6; ++i) kf.Q(i, i) = 1e-8;
    kf.R.setZero();
    for (int i = 0; i < 3; ++i) kf.R(i, i) = 100.0;
}

void configure(ExtendedKalmanFilter& ekf) {
    ekf.x = iss_state();
    ekf.P.setIdentity();
    ekf.P *= 100.0;
    ekf.Q.setZero();
    for (int i = 0; i < 3; ++i) ekf.Q(i, i) = 1e-4;
    for (int i = 3; i < 6; ++i) ekf.Q(i, i) = 1e-8;
    ekf.R.setZero();
    for (int i = 0; i < 3; ++i) ekf.R(i, i) = 100.0;
    ekf.perturb_cfg.enable_j2   = true;   // matches EKF's analytical Jacobian (gravity+J2)
    ekf.perturb_cfg.enable_drag = false;
    ekf.perturb_cfg.enable_srp  = false;
}

void configure(UnscentedKalmanFilter& ukf) {
    ukf.x = iss_state();
    ukf.P.setIdentity();
    ukf.P *= 100.0;
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(ukf.P);
    ukf.S = llt.matrixL();
    ukf.Q.setZero();
    for (int i = 0; i < 3; ++i) ukf.Q(i, i) = 1e-4;
    for (int i = 3; i < 6; ++i) ukf.Q(i, i) = 1e-8;
    ukf.R.setZero();
    for (int i = 0; i < 3; ++i) ukf.R(i, i) = 100.0;
    ukf.perturb_cfg.enable_j2   = true;
    ukf.perturb_cfg.enable_drag = false;
    ukf.perturb_cfg.enable_srp  = false;
}

} // namespace

// Zero-innovation measurement (z = predicted position) keeps the filter
// numerically stable over many iterations while still exercising the full
// predict+update matrix pipeline — only P changes step to step, x just
// follows the real two-body trajectory under predict().

double bench_kf_step(int iterations) {
    KalmanFilter kf;
    configure(kf);
    const auto t0 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        kf.predict(k_dt);
        kf.update(kf.x.head<3>());
    }
    const auto t1 = Clock::now();
    volatile double sink = kf.x.norm();
    (void)sink;
    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
}

double bench_ekf_step(int iterations) {
    ExtendedKalmanFilter ekf;
    configure(ekf);
    const auto t0 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        ekf.predict(k_dt);
        ekf.update(ekf.x.head<3>());
    }
    const auto t1 = Clock::now();
    volatile double sink = ekf.x.norm();
    (void)sink;
    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
}

double bench_ukf_step(int iterations) {
    UnscentedKalmanFilter ukf;
    configure(ukf);
    const auto t0 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        ukf.predict(k_dt);
        ukf.update(ukf.x.head<3>());
    }
    const auto t1 = Clock::now();
    volatile double sink = ukf.x.norm();
    (void)sink;
    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
}

double bench_three_filters_step(int iterations) {
    KalmanFilter kf;                configure(kf);
    ExtendedKalmanFilter ekf;        configure(ekf);
    UnscentedKalmanFilter ukf;       configure(ukf);

    const auto t0 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        kf.predict(k_dt);   kf.update(kf.x.head<3>());
        ekf.predict(k_dt);  ekf.update(ekf.x.head<3>());
        ukf.predict(k_dt);  ukf.update(ukf.x.head<3>());
    }
    const auto t1 = Clock::now();
    volatile double sink = kf.x.norm() + ekf.x.norm() + ukf.x.norm();
    (void)sink;
    return std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
}

void run_filter_benchmarks() {
    constexpr int N = 100'000;
    const double kf_us   = bench_kf_step(N);
    const double ekf_us  = bench_ekf_step(N);
    const double ukf_us  = bench_ukf_step(N);
    const double all3_us = bench_three_filters_step(N);

    std::printf("KF  predict+update (%d iterations):        %.3f us/step\n", N, kf_us);
    std::printf("EKF predict+update (%d iterations):        %.3f us/step\n", N, ekf_us);
    std::printf("UKF predict+update (%d iterations):        %.3f us/step\n", N, ukf_us);
    std::printf("3 filters simultaneously (%d iterations):  %.3f us/tick\n", N, all3_us);
}

} // namespace orbitforge::benchmarks
