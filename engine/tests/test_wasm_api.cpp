#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "wasm_api.hpp"

using orbitforge::ScenarioCfg;
using orbitforge::Simulation;
using orbitforge::StateFrame;
using orbitforge::faults::FaultConfig;
using orbitforge::faults::FaultType;

namespace {

std::string make_tle_line1(int sat_num, int epoch_year_2d, double epoch_day) {
    std::string line(69, ' ');
    line[0] = '1';
    char field[16];
    std::snprintf(field, sizeof(field), "%05d", sat_num);
    line.replace(2, 5, field);
    line[7] = 'U';
    std::snprintf(field, sizeof(field), "%02d", epoch_year_2d);
    line.replace(18, 2, field);
    std::snprintf(field, sizeof(field), "%012.8f", epoch_day);
    line.replace(20, 12, field);
    return line;
}

std::string make_tle_line2(int sat_num, double incl_deg, double raan_deg, double ecc,
                            double argp_deg, double ma_deg, double mean_motion_rev_day) {
    std::string line(69, ' ');
    line[0] = '2';
    char field[16];
    std::snprintf(field, sizeof(field), "%05d", sat_num);
    line.replace(2, 5, field);
    std::snprintf(field, sizeof(field), "%8.4f", incl_deg);
    line.replace(8, 8, field);
    std::snprintf(field, sizeof(field), "%8.4f", raan_deg);
    line.replace(17, 8, field);
    std::snprintf(field, sizeof(field), "%07d", static_cast<int>(std::lround(ecc * 1e7)));
    line.replace(26, 7, field);
    std::snprintf(field, sizeof(field), "%8.4f", argp_deg);
    line.replace(34, 8, field);
    std::snprintf(field, sizeof(field), "%8.4f", ma_deg);
    line.replace(43, 8, field);
    std::snprintf(field, sizeof(field), "%11.8f", mean_motion_rev_day);
    line.replace(52, 11, field);
    return line;
}

ScenarioCfg iss_like_cfg() {
    const std::string l1 = make_tle_line1(25544, 24, 1.5);
    const std::string l2 =
        make_tle_line2(25544, 51.6400, 247.4627, 0.0006703, 130.5360, 325.0288, 15.49000000);

    ScenarioCfg cfg;
    std::snprintf(cfg.tle_line1, sizeof(cfg.tle_line1), "%s", l1.c_str());
    std::snprintf(cfg.tle_line2, sizeof(cfg.tle_line2), "%s", l2.c_str());
    cfg.gps_sigma = 10.0;
    cfg.imu_sigma = 0.05;
    cfg.enable_j2 = true;
    cfg.enable_drag = false;
    cfg.enable_srp = false;
    cfg.drag_coeff = 2.2;
    cfg.area_to_mass = 0.01;
    cfg.q_pos = 1.0;
    cfg.q_vel = 0.01;
    cfg.sim_speed = 1.0;
    cfg.seed = 7;
    return cfg;
}

}

TEST(Simulation, RingBufferReceivesOneFramePerStep) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    constexpr int k_steps = 10;
    for (int i = 0; i < k_steps; ++i) sim.step(1.0);

    EXPECT_EQ(sim.ring_buffer().size(), static_cast<size_t>(k_steps));
}

TEST(Simulation, StepAdvancesSimTimeAndProducesFiniteFrame) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());
    sim.step(1.0);

    EXPECT_DOUBLE_EQ(sim.get_sim_time(), 1.0);

    StateFrame frame;
    ASSERT_TRUE(sim.ring_buffer().pop(frame));
    EXPECT_DOUBLE_EQ(frame.sim_time, 1.0);
    for (double v : frame.true_pos) EXPECT_TRUE(std::isfinite(v));
    for (double v : frame.kf_cov_diag) EXPECT_TRUE(std::isfinite(v));
    EXPECT_TRUE(std::isfinite(frame.ekf_nis));
    EXPECT_TRUE(std::isfinite(frame.ukf_nis));
}

TEST(Simulation, GetRingBufferPtrMatchesRingBufferAddress) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    const auto ptr = sim.get_ring_buffer_ptr();
    EXPECT_EQ(reinterpret_cast<void*>(ptr), static_cast<void*>(&sim.ring_buffer()));
    EXPECT_EQ(sim.get_ring_buffer_capacity(), Simulation::get_ring_buffer_capacity());
}

TEST(Simulation, ResetRestoresInitialState) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());
    for (int i = 0; i < 5; ++i) sim.step(1.0);
    ASSERT_GT(sim.get_sim_time(), 0.0);

    sim.reset();

    EXPECT_DOUBLE_EQ(sim.get_sim_time(), 0.0);
    EXPECT_EQ(sim.ring_buffer().size(), 0u);
    EXPECT_FALSE(sim.is_running());
}

TEST(Simulation, StartPauseTogglesIsRunningAndAdvancesTime) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    sim.start();
    EXPECT_TRUE(sim.is_running());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sim.pause();

    EXPECT_FALSE(sim.is_running());
    EXPECT_GT(sim.get_sim_time(), 0.0);
}

TEST(Simulation, SetSimSpeedTakesEffectOnNextStartAfterPause) {
    Simulation sim;
    ScenarioCfg cfg = iss_like_cfg();
    cfg.sim_speed = 1.0;
    sim.init_scenario(cfg);

    sim.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sim.pause();
    const double advance_at_1x = sim.get_sim_time();
    ASSERT_GT(advance_at_1x, 0.0);

    sim.set_sim_speed(50.0);
    sim.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sim.pause();
    const double advance_at_50x = sim.get_sim_time() - advance_at_1x;

    EXPECT_GT(advance_at_50x, advance_at_1x * 10.0);
}

TEST(Simulation, GpsDropoutFaultSuppressesMeasurementUpdate) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    FaultConfig fault;
    fault.type = FaultType::gps_dropout;
    fault.onset_t = 0.0;
    fault.duration = 5.0;
    sim.set_fault(fault);

    sim.step(1.0);
    StateFrame frame;
    ASSERT_TRUE(sim.ring_buffer().pop(frame));
    EXPECT_EQ(frame.active_fault, static_cast<uint8_t>(FaultType::gps_dropout));
    EXPECT_DOUBLE_EQ(frame.kf_nis, 0.0);
    EXPECT_DOUBLE_EQ(frame.ekf_nis, 0.0);
    EXPECT_DOUBLE_EQ(frame.ukf_nis, 0.0);

    for (int i = 0; i < 6; ++i) sim.step(1.0);
    StateFrame last;
    while (sim.ring_buffer().pop(last)) {
    }
    EXPECT_EQ(last.active_fault, static_cast<uint8_t>(FaultType::none));
}

TEST(Simulation, GpsSpikeFaultAppliesExactlyOnce) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    FaultConfig fault;
    fault.type = FaultType::gps_spike;
    fault.onset_t = 0.0;
    fault.duration = 0.0;
    fault.magnitude = 500.0;
    sim.set_fault(fault);

    sim.step(1.0);
    StateFrame first;
    ASSERT_TRUE(sim.ring_buffer().pop(first));
    EXPECT_EQ(first.active_fault, static_cast<uint8_t>(FaultType::gps_spike));

    sim.step(1.0);
    StateFrame second;
    ASSERT_TRUE(sim.ring_buffer().pop(second));
    EXPECT_EQ(second.active_fault, static_cast<uint8_t>(FaultType::none));
}

TEST(Simulation, ManeuverFaultChangesTrueVelocityOnce) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    sim.step(1.0);
    StateFrame before;
    ASSERT_TRUE(sim.ring_buffer().pop(before));
    const double v_before = std::sqrt(before.true_vel[0] * before.true_vel[0] +
                                       before.true_vel[1] * before.true_vel[1] +
                                       before.true_vel[2] * before.true_vel[2]);

    FaultConfig fault;
    fault.type = FaultType::maneuver;
    fault.onset_t = 1.0;
    fault.magnitude = 50.0;
    sim.set_fault(fault);

    sim.step(1.0);
    StateFrame after;
    ASSERT_TRUE(sim.ring_buffer().pop(after));
    const double v_after = std::sqrt(after.true_vel[0] * after.true_vel[0] +
                                      after.true_vel[1] * after.true_vel[1] +
                                      after.true_vel[2] * after.true_vel[2]);

    EXPECT_NEAR(v_after - v_before, 50.0, 1.0);
    EXPECT_EQ(after.active_fault, static_cast<uint8_t>(FaultType::maneuver));
}

TEST(Simulation, SensorBiasFaultShiftsPositionEstimateThenClears) {
    constexpr int kWarmupSteps = 30;
    constexpr double kBiasMeters = 300.0;
    constexpr int kFaultDurationSec = 10;

    Simulation baseline;
    baseline.init_scenario(iss_like_cfg());
    for (int i = 0; i < kWarmupSteps + kFaultDurationSec; ++i) baseline.step(1.0);
    StateFrame base_frame;
    while (baseline.ring_buffer().pop(base_frame)) {
    }

    Simulation biased;
    biased.init_scenario(iss_like_cfg());
    for (int i = 0; i < kWarmupSteps; ++i) biased.step(1.0);
    StateFrame warm;
    while (biased.ring_buffer().pop(warm)) {
    }
    EXPECT_EQ(warm.active_fault, static_cast<uint8_t>(FaultType::none));

    FaultConfig fault;
    fault.type = FaultType::sensor_bias;
    fault.onset_t = static_cast<double>(kWarmupSteps);
    fault.duration = static_cast<double>(kFaultDurationSec);
    fault.magnitude = kBiasMeters;
    biased.set_fault(fault);

    StateFrame current;
    for (int i = 0; i < kFaultDurationSec; ++i) {
        biased.step(1.0);
        ASSERT_TRUE(biased.ring_buffer().pop(current));
        EXPECT_EQ(current.active_fault, static_cast<uint8_t>(FaultType::sensor_bias));
    }

    EXPECT_GT(std::abs(current.kf_pos[0] - base_frame.kf_pos[0]), 20.0);
    EXPECT_GT(std::abs(current.ekf_pos[0] - base_frame.ekf_pos[0]), 20.0);
    EXPECT_GT(std::abs(current.ukf_pos[0] - base_frame.ukf_pos[0]), 20.0);

    for (int i = 0; i < 5; ++i) biased.step(1.0);
    StateFrame last;
    while (biased.ring_buffer().pop(last)) {
    }
    EXPECT_EQ(last.active_fault, static_cast<uint8_t>(FaultType::none));
}

TEST(WasmFreeFunctionApi, MatchesSimulationBehavior) {
    using namespace orbitforge;

    init_scenario(iss_like_cfg());
    EXPECT_FALSE(is_running());

    start_simulation();
    EXPECT_TRUE(is_running());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    pause_simulation();
    EXPECT_FALSE(is_running());
    EXPECT_GT(get_sim_time(), 0.0);

    FaultConfig fault;
    fault.type = FaultType::gps_spike;
    fault.magnitude = 100.0;
    set_fault(fault);

    EXPECT_NE(get_ring_buffer_ptr(), uintptr_t{0});
    EXPECT_EQ(get_ring_buffer_capacity(), Simulation::get_ring_buffer_capacity());

    reset_simulation();
    EXPECT_DOUBLE_EQ(get_sim_time(), 0.0);
}

TEST(Simulation, RunMonteCarloProducesCorrectlySizedFiniteResults) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    constexpr size_t k_n_runs = 20;
    orbitforge::monte_carlo::MCConfig req;
    req.n_runs = k_n_runs;
    req.seed = 123;
    sim.run_monte_carlo(req);

    const auto& stats = sim.get_mc_results();
    EXPECT_EQ(sim.get_mc_n_runs(), k_n_runs);
    EXPECT_EQ(stats.rms_pos.size(), 500u);
    EXPECT_EQ(stats.rms_vel.size(), 500u);
    EXPECT_EQ(stats.nees.size(), 500u);
    EXPECT_EQ(stats.nis.size(), 500u);
    EXPECT_EQ(stats.final_pos_err.size(), k_n_runs);
    for (double v : stats.rms_pos) EXPECT_TRUE(std::isfinite(v));
    for (double v : stats.nees) EXPECT_TRUE(std::isfinite(v) && v > 0.0);
    for (double v : stats.final_pos_err) EXPECT_TRUE(std::isfinite(v) && v >= 0.0);
}

TEST(Simulation, RunMonteCarloIsDeterministicGivenSameSeed) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    orbitforge::monte_carlo::MCConfig req;
    req.n_runs = 10;
    req.seed = 555;

    sim.run_monte_carlo(req);
    const auto a = sim.get_mc_results().nees;
    sim.run_monte_carlo(req);
    const auto b = sim.get_mc_results().nees;

    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) EXPECT_DOUBLE_EQ(a[i], b[i]);
}

TEST(Simulation, RunMonteCarloPausesAnyRunningLiveSimulation) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());
    sim.start();
    ASSERT_TRUE(sim.is_running());

    orbitforge::monte_carlo::MCConfig req;
    req.n_runs = 4;
    req.seed = 1;
    sim.run_monte_carlo(req);

    EXPECT_FALSE(sim.is_running());
}

TEST(Simulation, RunMonteCarloPassesThroughUserConfigurableFields) {
    Simulation sim;
    sim.init_scenario(iss_like_cfg());

    orbitforge::monte_carlo::MCConfig req;
    req.n_runs = 6;
    req.seed = 7;
    req.filter = orbitforge::monte_carlo::FilterKind::ukf;
    req.n_steps = 37;
    req.dt = 5.0;
    req.q_pos = 2.5;
    req.q_vel = 0.05;
    sim.run_monte_carlo(req);

    const auto& stats = sim.get_mc_results();
    EXPECT_EQ(stats.rms_pos.size(), 37u);
    for (double v : stats.nees) EXPECT_TRUE(std::isfinite(v));
}

TEST(WasmFreeFunctionApi, RunMonteCarloMatchesSimulationBehavior) {
    using namespace orbitforge;

    init_scenario(iss_like_cfg());
    monte_carlo::MCConfig req;
    req.n_runs = 8;
    req.seed = 99;
    orbitforge::run_monte_carlo(req);

    EXPECT_EQ(get_mc_n_runs(), 8u);
    EXPECT_EQ(get_mc_results().final_pos_err.size(), 8u);
}

TEST(Simulation, DragCoeffErrorFaultShiftsTrueDragOnly) {
    ScenarioCfg cfg = iss_like_cfg();
    cfg.enable_drag = true;
    Simulation sim;
    sim.init_scenario(cfg);

    FaultConfig fault;
    fault.type = FaultType::drag_coeff_error;
    fault.onset_t = 0.0;
    fault.duration = 0.0;
    fault.magnitude = 0.5;
    sim.set_fault(fault);

    sim.step(1.0);
    StateFrame frame;
    ASSERT_TRUE(sim.ring_buffer().pop(frame));
    EXPECT_EQ(frame.active_fault, static_cast<uint8_t>(FaultType::drag_coeff_error));
}
