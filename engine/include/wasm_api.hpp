#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include <Eigen/Dense>

#include "constants.hpp"
#include "dynamics/eom.hpp"
#include "faults/fault_injector.hpp"
#include "filters/ekf.hpp"
#include "filters/kf.hpp"
#include "filters/ukf.hpp"
#include "memory/ring_buffer.hpp"
#include "scenario.hpp"
#include "sensors/gps.hpp"

namespace orbitforge {

// One slot written to the ring buffer per simulation tick (CLAUDE.md §21).
// All fields are double except the trailing uint8_t, so the compiler's
// default alignment already makes sizeof(StateFrame) a multiple of 8 —
// no manual padding needed.
struct StateFrame {
    double sim_time = 0.0;

    double true_pos[3] = {};
    double true_vel[3] = {};

    double kf_pos[3] = {};
    double kf_vel[3] = {};
    double kf_cov_diag[6] = {};
    double kf_nis = 0.0;

    double ekf_pos[3] = {};
    double ekf_vel[3] = {};
    double ekf_cov_diag[6] = {};
    double ekf_nis = 0.0;

    double ukf_pos[3] = {};
    double ukf_vel[3] = {};
    double ukf_cov_diag[6] = {};
    double ukf_nis = 0.0;

    uint8_t active_fault = 0;
};

constexpr size_t k_ring_buffer_capacity = 512;

// Ties together the true trajectory, all 3 filters, the GPS sensor model,
// and fault injection into the single simulation loop described in
// CLAUDE.md §4/§21. Owns the SPSCRingBuffer the renderer reads from.
//
// step() is the deterministic, directly-testable core: true-trajectory RK4
// propagation, filter predict, GPS sense+update, fault application, and one
// ring-buffer push. start()/pause() just drive step() repeatedly from a
// background thread at a fixed wall-clock cadence; nothing in step() itself
// depends on wall-clock time, so it can be unit-tested by calling it
// directly with an arbitrary dt.
class Simulation {
public:
    Simulation();
    ~Simulation();

    void init_scenario(const ScenarioCfg& cfg);
    void start();
    void pause();
    void reset();
    void set_fault(const faults::FaultConfig& cfg);

    void step(double dt);

    double get_sim_time() const noexcept { return sim_time_.load(std::memory_order_relaxed); }
    bool   is_running() const noexcept { return running_.load(std::memory_order_relaxed); }

    uintptr_t get_ring_buffer_ptr() const noexcept {
        return reinterpret_cast<uintptr_t>(&ring_buffer_);
    }
    static constexpr size_t get_ring_buffer_capacity() noexcept { return k_ring_buffer_capacity; }

    // Exposed for tests; not part of the CLAUDE.md §21 public API.
    memory::SPSCRingBuffer<StateFrame, k_ring_buffer_capacity>& ring_buffer() noexcept {
        return ring_buffer_;
    }

private:
    void run_loop();
    void init_filters();

    ScenarioCfg cfg_;
    double      epoch_jd_ = k_j2000_jd;

    Eigen::Matrix<double, 6, 1>  x_true_;
    dynamics::PerturbationConfig perturb_true_;
    dynamics::PerturbationConfig perturb_nominal_;

    filters::KalmanFilter         kf_;
    filters::ExtendedKalmanFilter  ekf_;
    filters::UnscentedKalmanFilter ukf_;
    sensors::GpsSensor              gps_;

    faults::FaultQueue  fault_queue_;
    faults::FaultConfig active_fault_;
    bool                fault_applied_once_ = false;

    std::atomic<double> sim_time_{0.0};
    std::atomic<bool>   running_{false};
    std::thread         worker_thread_;

    memory::SPSCRingBuffer<StateFrame, k_ring_buffer_capacity> ring_buffer_;
};

// Free-function API matching CLAUDE.md §21 exactly, operating on a single
// global Simulation instance — this is the surface EMSCRIPTEN_BINDINGS
// exposes to JS (see wasm_api.cpp).
void      init_scenario(const ScenarioCfg& cfg);
void      start_simulation();
void      pause_simulation();
void      reset_simulation();
uintptr_t get_ring_buffer_ptr();
size_t    get_ring_buffer_capacity();
void      set_fault(const faults::FaultConfig& fault);
double    get_sim_time();
bool      is_running();

}  // namespace orbitforge
