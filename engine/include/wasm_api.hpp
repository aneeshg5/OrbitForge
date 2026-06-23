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

struct StateFrame {
    double sim_time = 0.0;

    double true_pos[3] = {};
    double true_vel[3] = {};
    double true_quat[4] = {};
    double true_omega[3] = {};

    double kf_pos[3] = {};
    double kf_vel[3] = {};
    double kf_cov_diag[6] = {};
    double kf_nis = 0.0;

    double ekf_pos[3] = {};
    double ekf_vel[3] = {};
    double ekf_quat[4] = {};
    double ekf_omega[3] = {};
    double ekf_cov_diag[12] = {};
    double ekf_nis = 0.0;

    double ukf_pos[3] = {};
    double ukf_vel[3] = {};
    double ukf_quat[4] = {};
    double ukf_omega[3] = {};
    double ukf_cov_diag[12] = {};
    double ukf_nis = 0.0;

    uint8_t active_fault = 0;
};

constexpr size_t k_ring_buffer_capacity = 512;

class Simulation {
public:
    Simulation();
    ~Simulation();

    void init_scenario(const ScenarioCfg& cfg);
    void start();
    void pause();
    void reset();
    void set_fault(const faults::FaultConfig& cfg);

    // Only safe while paused/idle: run_loop() reads cfg_.sim_speed every
    // tick with no synchronization, so calling this while running() is a
    // data race (debug-asserted).
    void set_sim_speed(double sim_speed);

    void step(double dt);

    void run_monte_carlo(const monte_carlo::MCConfig& req_cfg);

    const monte_carlo::MCStats& get_mc_results() const noexcept { return mc_stats_; }
    size_t get_mc_n_runs() const noexcept { return mc_n_runs_; }

    double get_sim_time() const noexcept { return sim_time_.load(std::memory_order_relaxed); }
    bool   is_running() const noexcept { return running_.load(std::memory_order_relaxed); }

    uintptr_t get_ring_buffer_ptr() const noexcept {
        return reinterpret_cast<uintptr_t>(&ring_buffer_);
    }
    static constexpr size_t get_ring_buffer_capacity() noexcept { return k_ring_buffer_capacity; }

    memory::SPSCRingBuffer<StateFrame, k_ring_buffer_capacity>& ring_buffer() noexcept {
        return ring_buffer_;
    }

private:
    void run_loop();
    void init_filters();

    ScenarioCfg cfg_;
    double      epoch_jd_ = k_j2000_jd;

    Eigen::Matrix<double, 6, 1>  x_true_;
    Eigen::Matrix<double, 6, 1>  x_true_initial_;
    dynamics::PerturbationConfig perturb_true_;
    dynamics::PerturbationConfig perturb_nominal_;

    // math.md §7.2.
    dynamics::AttitudeState x_true_att_;
    dynamics::InertiaTensor inertia_;

    monte_carlo::MCStats mc_stats_;
    size_t                mc_n_runs_ = 0;

    filters::KalmanFilter         kf_;
    filters::ExtendedKalmanFilter  ekf_;
    filters::UnscentedKalmanFilter ukf_;
    sensors::GpsSensor              gps_;
    sensors::GyroSensor              gyro_;
    sensors::MagnetometerSensor      mag_;

    faults::FaultQueue  fault_queue_;
    faults::FaultConfig active_fault_;
    bool                fault_applied_once_ = false;

    std::atomic<double> sim_time_{0.0};
    std::atomic<bool>   running_{false};
    std::thread         worker_thread_;

    memory::SPSCRingBuffer<StateFrame, k_ring_buffer_capacity> ring_buffer_;
};

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

}
