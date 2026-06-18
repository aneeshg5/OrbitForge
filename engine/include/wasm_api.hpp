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
#include "monte_carlo/mc_runner.hpp"
#include "scenario.hpp"
#include "sensors/gps.hpp"

namespace orbitforge {

// One slot written to the ring buffer per simulation tick. All fields are
// double except the trailing uint8_t, so the compiler's
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
// and fault injection into a single simulation loop. Owns the SPSCRingBuffer
// the renderer reads from.
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

    // Runs an EKF Monte Carlo consistency campaign against the scenario's
    // initial true state (the state at the moment init_scenario() was last
    // called, not the live mid-simulation x_true_ — a stable, reproducible
    // initial condition for a campaign that may be run before, after, or
    // independent of the live single-run simulation). Always EKF: KF is
    // the intentionally-divergent demo filter and UKF is ~2.5x the
    // per-step cost (docs/benchmarks.md) for the same consistency question
    // EKF already answers, and the public API's run_monte_carlo(n_runs,
    // seed) signature has no filter-selection parameter, so this is the
    // one defensible default rather than an arbitrary one. n_steps=500/
    // dt=10s matches the validated setup in test_filter_consistency.cpp
    // (McRunner.EkfNeesConsistencyMatchesPhase1Result) — one ISS-orbit-scale
    // consistency check (~83 min of sim time), not configurable since
    // ScenarioCfg deliberately carries no duration field. Pauses any
    // running live simulation first: run_monte_carlo() spawns its own
    // monte_carlo::k_mc_threads (4) worker threads, and the WASM build's
    // PTHREAD_POOL_SIZE is sized to match that — running concurrently with
    // the live sim's own background thread would need a 5th pool slot.
    void run_monte_carlo(size_t n_runs, int seed);

    const monte_carlo::MCStats& get_mc_results() const noexcept { return mc_stats_; }
    size_t get_mc_n_runs() const noexcept { return mc_n_runs_; }

    double get_sim_time() const noexcept { return sim_time_.load(std::memory_order_relaxed); }
    bool   is_running() const noexcept { return running_.load(std::memory_order_relaxed); }

    uintptr_t get_ring_buffer_ptr() const noexcept {
        return reinterpret_cast<uintptr_t>(&ring_buffer_);
    }
    static constexpr size_t get_ring_buffer_capacity() noexcept { return k_ring_buffer_capacity; }

    // Exposed for tests; not part of the public WASM API.
    memory::SPSCRingBuffer<StateFrame, k_ring_buffer_capacity>& ring_buffer() noexcept {
        return ring_buffer_;
    }

private:
    void run_loop();
    void init_filters();

    ScenarioCfg cfg_;
    double      epoch_jd_ = k_j2000_jd;

    Eigen::Matrix<double, 6, 1>  x_true_;
    Eigen::Matrix<double, 6, 1>  x_true_initial_;  // snapshot at init_scenario(), for run_monte_carlo()
    dynamics::PerturbationConfig perturb_true_;
    dynamics::PerturbationConfig perturb_nominal_;

    monte_carlo::MCStats mc_stats_;
    size_t                mc_n_runs_ = 0;

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

// Free-function API operating on a single global Simulation instance —
// this is the surface EMSCRIPTEN_BINDINGS exposes to JS (see wasm_api.cpp).
void      init_scenario(const ScenarioCfg& cfg);
void      start_simulation();
void      pause_simulation();
void      reset_simulation();
uintptr_t get_ring_buffer_ptr();
size_t    get_ring_buffer_capacity();
void      set_fault(const faults::FaultConfig& fault);
double    get_sim_time();
bool      is_running();

void   run_monte_carlo(size_t n_runs, int seed);
const monte_carlo::MCStats& get_mc_results();
size_t get_mc_n_runs();

}  // namespace orbitforge
