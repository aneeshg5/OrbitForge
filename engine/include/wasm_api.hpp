#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include <Eigen/Dense>

#include "constants.hpp"
#include "dynamics/eom.hpp"
#include "dynamics/rigid_body.hpp"
#include "faults/fault_injector.hpp"
#include "filters/ekf.hpp"
#include "filters/kf.hpp"
#include "filters/ukf.hpp"
#include "math/quaternion.hpp"
#include "memory/ring_buffer.hpp"
#include "monte_carlo/mc_runner.hpp"
#include "scenario.hpp"
#include "sensors/gps.hpp"
#include "sensors/gyro.hpp"
#include "sensors/magnetometer.hpp"

namespace orbitforge {

// One slot written to the ring buffer per simulation tick. All fields are
// double except the trailing uint8_t, so the compiler's
// default alignment already makes sizeof(StateFrame) a multiple of 8 —
// no manual padding needed.
struct StateFrame {
    double sim_time = 0.0;

    double true_pos[3] = {};
    double true_vel[3] = {};
    double true_quat[4] = {};   // Phase 5: ECI<-body, Eigen::Quaterniond::coeffs() order (x,y,z,w)
    double true_omega[3] = {};  // Phase 5: body-frame rad/s

    // KF: UNCHANGED by Phase 5 — no attitude fields at all (deliberately
    // absent, not zeroed — see CLAUDE.md §6.1/§21: KF doesn't carry an
    // attitude state, so there is nothing to report here).
    double kf_pos[3] = {};
    double kf_vel[3] = {};
    double kf_cov_diag[6] = {};
    double kf_nis = 0.0;

    // EKF: Phase 5 grows this from 6-state to 12-state MEKF.
    double ekf_pos[3] = {};
    double ekf_vel[3] = {};
    double ekf_quat[4] = {};      // q_ref AFTER reset_attitude_error() — the actual attitude estimate
    double ekf_omega[3] = {};
    double ekf_cov_diag[12] = {}; // [delta_theta(3), omega(3), r(3), v(3)] diagonal
    double ekf_nis = 0.0;         // combined innovation across whichever of {GPS, gyro, mag} updated this tick

    // UKF: same Phase 5 growth as EKF.
    double ukf_pos[3] = {};
    double ukf_vel[3] = {};
    double ukf_quat[4] = {};
    double ukf_omega[3] = {};
    double ukf_cov_diag[12] = {};
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

    // Changes the playback rate for the *next* start()/resume, without
    // touching x_true_, filter state, or sim_time — unlike reset(), this
    // doesn't restart the scenario. Only safe while paused/idle: run_loop()
    // reads cfg_.sim_speed every tick with no synchronization, since
    // nothing else has ever mutated cfg_ after init_scenario() while the
    // worker thread could be running concurrently. Calling this while
    // running() is a data race (debug-asserted, not handled at runtime —
    // CLAUDE.md §20: assertions for preconditions, not hot-path checks).
    void set_sim_speed(double sim_speed);

    void step(double dt);

    // Runs a Monte Carlo consistency campaign against the scenario's
    // initial true state (the state at the moment init_scenario() was last
    // called, not the live mid-simulation x_true_ — a stable, reproducible
    // initial condition for a campaign that may be run before, after, or
    // independent of the live single-run simulation). req_cfg carries every
    // user-facing knob (n_runs, filter, n_steps, dt, q_pos, q_vel, seed) —
    // this overwrites only x0 and gps_sigma from live scenario state before
    // delegating to monte_carlo::run_monte_carlo(), since those two aren't
    // meant to be independently configurable in the MC panel (gps_sigma
    // already has a home in the Scenario Editor; x0 has no UI at all,
    // §6 "real TLE data" is the only source for it). Pauses any running
    // live simulation first: run_monte_carlo() spawns its own
    // monte_carlo::k_mc_threads (4) worker threads, and the WASM build's
    // PTHREAD_POOL_SIZE is sized to match that — running concurrently with
    // the live sim's own background thread would need a 5th pool slot.
    void run_monte_carlo(const monte_carlo::MCConfig& req_cfg);

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

    // Phase 5: "true" attitude trajectory — independent RK4 integration
    // (rigid_body.hpp's AttitudeState = [q.coeffs(), omega]), fully
    // decoupled from x_true_ above (torque-free rotation doesn't depend
    // on orbital state or vice versa — math.md §7.2). Not snapshotted for
    // run_monte_carlo(): that campaign stays orbital-only by design
    // (mc_runner.cpp's file doc) and never touches attitude.
    dynamics::AttitudeState x_true_att_;
    dynamics::InertiaTensor inertia_;

    monte_carlo::MCStats mc_stats_;
    size_t                mc_n_runs_ = 0;

    filters::KalmanFilter         kf_;
    filters::ExtendedKalmanFilter  ekf_;
    filters::UnscentedKalmanFilter ukf_;
    sensors::GpsSensor              gps_;
    sensors::GyroSensor              gyro_;   // Phase 5
    sensors::MagnetometerSensor      mag_;    // Phase 5

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
void      set_sim_speed(double sim_speed);
double    get_sim_time();
bool      is_running();

void   run_monte_carlo(const monte_carlo::MCConfig& req_cfg);
const monte_carlo::MCStats& get_mc_results();
size_t get_mc_n_runs();

}  // namespace orbitforge
