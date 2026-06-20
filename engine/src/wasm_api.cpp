#include "wasm_api.hpp"

#include <chrono>
#include <thread>

#include "integrators/rk4.hpp"

namespace orbitforge {

namespace {

void seed_filter(filters::FilterBase<6, 3>& filter, const Eigen::Matrix<double, 6, 1>& x0,
                  double q_pos, double q_vel, double gps_sigma) {
    constexpr double k_p0_pos = 100.0 * 100.0;  // 100 m std dev, squared
    constexpr double k_p0_vel = 1.0 * 1.0;      // 1 m/s std dev, squared

    filter.x = x0;
    filter.P.setZero();
    filter.P.diagonal().head<3>().setConstant(k_p0_pos);
    filter.P.diagonal().tail<3>().setConstant(k_p0_vel);
    filter.Q.setZero();
    filter.Q.diagonal().head<3>().setConstant(q_pos * q_pos);
    filter.Q.diagonal().tail<3>().setConstant(q_vel * q_vel);
    filter.R = Eigen::Matrix3d::Identity() * (gps_sigma * gps_sigma);
}

// Phase 5: seeds EKF/UKF's 12-state MEKF — [delta_theta(0-2), omega(3-5),
// r(6-8), v(9-11)]. Initial attitude uncertainty (30 deg, 0.01 rad/s) is a
// fixed default, not yet user-configurable (no UI control for it exists —
// matches the existing precedent of p0_pos/p0_vel above also being fixed
// constants rather than ScenarioCfg fields). R is set to the GPS default
// here; Simulation::step() overwrites H/R before each individual
// update() call (GPS, gyro, magnetometer all share one R slot per filter,
// swapped per measurement type — see FilterBase's doc).
template <typename Filter>
void seed_filter_6dof(Filter& filter, const Eigen::Matrix<double, 6, 1>& x0, const ScenarioCfg& cfg) {
    constexpr double k_p0_att   = 0.3 * 0.3;     // ~17 deg std dev, squared (rad^2)
    constexpr double k_p0_omega = 0.01 * 0.01;   // rad/s, squared
    constexpr double k_p0_pos   = 100.0 * 100.0;
    constexpr double k_p0_vel   = 1.0 * 1.0;

    filter.x.setZero();
    filter.x.template segment<3>(6) = x0.template head<3>();
    filter.x.template segment<3>(9) = x0.template tail<3>();
    filter.P.setIdentity();  // overwritten on-diagonal below; off-diagonal stays 0
    filter.P.diagonal().template segment<3>(0).setConstant(k_p0_att);
    filter.P.diagonal().template segment<3>(3).setConstant(k_p0_omega);
    filter.P.diagonal().template segment<3>(6).setConstant(k_p0_pos);
    filter.P.diagonal().template segment<3>(9).setConstant(k_p0_vel);
    filter.Q.setZero();
    filter.Q.diagonal().template segment<3>(0).setConstant(cfg.q_att * cfg.q_att);
    filter.Q.diagonal().template segment<3>(3).setConstant(cfg.q_omega * cfg.q_omega);
    filter.Q.diagonal().template segment<3>(6).setConstant(cfg.q_pos * cfg.q_pos);
    filter.Q.diagonal().template segment<3>(9).setConstant(cfg.q_vel * cfg.q_vel);
    filter.R = Eigen::Matrix3d::Identity() * (cfg.gps_sigma * cfg.gps_sigma);
    filter.inertia = dynamics::InertiaTensor{cfg.inertia_x, cfg.inertia_y, cfg.inertia_z};
    filter.q_ref = math::Quat::Identity();
}

}  // namespace

Simulation::Simulation() = default;

Simulation::~Simulation() {
    running_.store(false, std::memory_order_relaxed);
    if (worker_thread_.joinable()) worker_thread_.join();
}

void Simulation::init_filters() {
    seed_filter(kf_, x_true_, cfg_.q_pos, cfg_.q_vel, cfg_.gps_sigma);
    seed_filter_6dof(ekf_, x_true_, cfg_);
    seed_filter_6dof(ukf_, x_true_, cfg_);

    ekf_.perturb_cfg = perturb_nominal_;
    ekf_.julian_date = epoch_jd_;
    ukf_.perturb_cfg = perturb_nominal_;
    ukf_.julian_date = epoch_jd_;
    ukf_.S = ukf_.P.llt().matrixL();

    // Phase 5: "true" attitude trajectory, independent of x_true_ (math.md
    // §7.2 — torque-free rotation doesn't depend on orbital state).
    inertia_ = dynamics::InertiaTensor{cfg_.inertia_x, cfg_.inertia_y, cfg_.inertia_z};
    x_true_att_.head<4>() = math::Quat::Identity().coeffs();
    x_true_att_.tail<3>() = Eigen::Vector3d(cfg_.init_omega_x, cfg_.init_omega_y, cfg_.init_omega_z);

    const unsigned base_seed = cfg_.seed >= 0 ? static_cast<unsigned>(cfg_.seed) : 42u;
    gyro_ = sensors::GyroSensor(cfg_.gyro_sigma, 0.0005, base_seed + 1u);
    mag_  = sensors::MagnetometerSensor(cfg_.mag_sigma, base_seed + 2u);
}

void Simulation::init_scenario(const ScenarioCfg& cfg) {
    pause();  // stop any running loop before reconfiguring

    cfg_ = cfg;

    const TleElements elem = parse_tle(cfg.tle_line1, cfg.tle_line2);
    epoch_jd_ = elem.epoch_jd;
    const EciState initial = tle_elements_to_eci(elem);
    x_true_.head<3>() = initial.pos;
    x_true_.tail<3>() = initial.vel;
    x_true_initial_ = x_true_;

    perturb_nominal_.enable_j2    = cfg.enable_j2;
    perturb_nominal_.enable_drag  = cfg.enable_drag;
    perturb_nominal_.enable_srp   = cfg.enable_srp;
    perturb_nominal_.drag_coeff   = cfg.drag_coeff;
    perturb_nominal_.area_to_mass = cfg.area_to_mass;
    perturb_true_ = perturb_nominal_;

    gps_ = sensors::GpsSensor(cfg.gps_sigma, cfg.seed >= 0 ? static_cast<unsigned>(cfg.seed) : 42u);

    sim_time_.store(0.0, std::memory_order_relaxed);
    active_fault_ = faults::FaultConfig{};
    fault_applied_once_ = false;
    faults::FaultConfig drained;
    while (fault_queue_.try_take(drained)) {
        // drop any fault left over from a previous scenario
    }

    init_filters();
    ring_buffer_.clear();
}

void Simulation::start() {
    if (running_.load(std::memory_order_relaxed)) return;
    if (worker_thread_.joinable()) worker_thread_.join();
    running_.store(true, std::memory_order_relaxed);
    worker_thread_ = std::thread(&Simulation::run_loop, this);
}

void Simulation::pause() {
    running_.store(false, std::memory_order_relaxed);
    if (worker_thread_.joinable()) worker_thread_.join();
}

void Simulation::reset() {
    init_scenario(cfg_);
}

void Simulation::set_fault(const faults::FaultConfig& cfg) {
    fault_queue_.set(cfg);
}

void Simulation::run_monte_carlo(size_t n_runs, int seed) {
    pause();  // see run_monte_carlo()'s declaration comment in wasm_api.hpp

    monte_carlo::MCConfig mc_cfg;
    mc_cfg.n_runs  = n_runs;
    mc_cfg.n_steps = 500;
    mc_cfg.dt      = 10.0;
    mc_cfg.gps_sigma = cfg_.gps_sigma;
    mc_cfg.q_pos     = cfg_.q_pos;
    mc_cfg.q_vel     = cfg_.q_vel;
    mc_cfg.filter    = monte_carlo::FilterKind::ekf;
    mc_cfg.seed      = (seed >= 0) ? static_cast<unsigned>(seed) : 42u;
    mc_cfg.x0        = x_true_initial_;

    mc_stats_  = monte_carlo::run_monte_carlo(mc_cfg);
    mc_n_runs_ = n_runs;
}

void Simulation::run_loop() {
    constexpr double k_tick_hz = 100.0;
    const auto tick_period = std::chrono::duration<double>(1.0 / k_tick_hz);

    auto next_tick = std::chrono::steady_clock::now();
    while (running_.load(std::memory_order_relaxed)) {
        step((1.0 / k_tick_hz) * cfg_.sim_speed);

        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(tick_period);
        std::this_thread::sleep_until(next_tick);
    }
}

void Simulation::step(double dt) {
    const double t_now = sim_time_.load(std::memory_order_relaxed);

    faults::FaultConfig new_fault;
    if (fault_queue_.try_take(new_fault)) {
        active_fault_ = new_fault;
        fault_applied_once_ = false;
        perturb_true_.drag_coeff = perturb_nominal_.drag_coeff;  // clear any prior drag fault
    }

    auto reported_fault = faults::FaultType::none;
    bool gps_dropout_active = false;
    double gps_spike_offset = 0.0;
    bool apply_maneuver_now = false;

    if (active_fault_.type != faults::FaultType::none && t_now >= active_fault_.onset_t) {
        switch (active_fault_.type) {
            case faults::FaultType::gps_spike:
                if (!fault_applied_once_) {
                    gps_spike_offset = active_fault_.magnitude;
                    reported_fault = faults::FaultType::gps_spike;
                    fault_applied_once_ = true;
                }
                break;
            case faults::FaultType::gps_dropout:
                if (t_now < active_fault_.onset_t + active_fault_.duration) {
                    gps_dropout_active = true;
                    reported_fault = faults::FaultType::gps_dropout;
                }
                break;
            case faults::FaultType::maneuver:
                if (!fault_applied_once_) {
                    apply_maneuver_now = true;
                    reported_fault = faults::FaultType::maneuver;
                    fault_applied_once_ = true;
                }
                break;
            case faults::FaultType::drag_coeff_error: {
                const bool persists = (active_fault_.duration <= 0.0) ||
                    (t_now < active_fault_.onset_t + active_fault_.duration);
                perturb_true_.drag_coeff = persists
                    ? perturb_nominal_.drag_coeff * (1.0 + active_fault_.magnitude)
                    : perturb_nominal_.drag_coeff;
                if (persists) reported_fault = faults::FaultType::drag_coeff_error;
                break;
            }
            case faults::FaultType::sensor_bias: {
                // No observable effect yet: the filters are GPS-position-only
                // and don't fuse IMU measurements. The fault is still
                // tracked/reported here so the UI can show it queued.
                const bool persists = (active_fault_.duration <= 0.0) ||
                    (t_now < active_fault_.onset_t + active_fault_.duration);
                if (persists) reported_fault = faults::FaultType::sensor_bias;
                break;
            }
            case faults::FaultType::none:
                break;
        }
    }

    // True trajectory: deterministic RK4 + perturbations — this is ground truth.
    const double jd_now = epoch_jd_ + t_now / k_sec_per_day;
    const dynamics::PerturbationConfig true_cfg = perturb_true_;
    auto true_dyn = [&true_cfg, jd_now](double /*t*/, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), jd_now, true_cfg);
        return ds;
    };
    x_true_ = rk4_step(x_true_, 0.0, dt, true_dyn);

    if (apply_maneuver_now) {
        // Impulsive delta-v applied prograde (along current velocity),
        // unseen by the filters — they only ever see the GPS measurement.
        x_true_.tail<3>() += active_fault_.magnitude * x_true_.tail<3>().normalized();
    }

    // Phase 5: "true" attitude trajectory — independent RK4 step (math.md
    // §7.2), fully decoupled from x_true_ above.
    {
        const dynamics::InertiaTensor inertia = inertia_;
        auto att_dyn = [&inertia](double /*t*/, const dynamics::AttitudeState& s) {
            return dynamics::attitude_derivative(s, inertia);
        };
        x_true_att_ = rk4_step(x_true_att_, 0.0, dt, att_dyn);
        const math::Quat q_true = math::Quat(x_true_att_[3], x_true_att_[0], x_true_att_[1], x_true_att_[2]).normalized();
        x_true_att_.head<4>() = q_true.coeffs();
    }
    const math::Quat q_true(x_true_att_[3], x_true_att_[0], x_true_att_[1], x_true_att_[2]);
    const Eigen::Vector3d omega_true = x_true_att_.tail<3>();

    kf_.predict(dt);
    ekf_.predict(dt);
    ukf_.predict(dt);

    StateFrame frame;
    frame.sim_time = t_now + dt;
    for (int i = 0; i < 3; ++i) {
        frame.true_pos[i] = x_true_(i);
        frame.true_vel[i] = x_true_(i + 3);
        frame.true_omega[i] = omega_true(i);
    }
    for (int i = 0; i < 4; ++i) frame.true_quat[i] = q_true.coeffs()(i);

    if (!gps_dropout_active) {
        const Eigen::Matrix3d R_ecef_eci = sensors::GpsSensor::R_ecef_eci(jd_now);
        Eigen::Vector3d z = gps_.measure(x_true_.head<3>(), jd_now);
        if (gps_spike_offset != 0.0) {
            z += Eigen::Vector3d(gps_spike_offset, 0.0, 0.0);
        }

        Eigen::Matrix<double, 3, 6> H_kf;
        H_kf << R_ecef_eci, Eigen::Matrix3d::Zero();
        Eigen::Matrix<double, 3, 12> H_6dof = Eigen::Matrix<double, 3, 12>::Zero();
        H_6dof.block<3, 3>(0, 6) = R_ecef_eci;

        kf_.H  = H_kf;
        ekf_.H = H_6dof;
        ukf_.H = H_6dof;

        auto nis = [&z](const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                         const Eigen::MatrixXd& H, const Eigen::Matrix3d& R) {
            const Eigen::Vector3d nu = z - H * x;
            const Eigen::Matrix3d S = H * P * H.transpose() + R;
            return nu.dot(S.ldlt().solve(nu));
        };
        // NIS uses dynamically-sized Eigen::MatrixXd/VectorXd views purely
        // to share one lambda across KF's 6-state and EKF/UKF's 12-state —
        // a deliberate, deliberately-contained exception to §20's "never
        // MatrixXd in engine/" rule: this runs once per tick (not the hot
        // predict/update path) and avoids three near-duplicate copies of
        // the same five-line computation.
        frame.kf_nis  = nis(kf_.x, kf_.P, H_kf, kf_.R);
        frame.ekf_nis = nis(ekf_.x, ekf_.P, H_6dof, ekf_.R);
        frame.ukf_nis = nis(ukf_.x, ukf_.P, H_6dof, ukf_.R);

        kf_.update(z);
        ekf_.update(z);
        ukf_.update(z);
    }

    // Phase 5: gyro + magnetometer updates, EKF/UKF only (KF has no
    // attitude state to update, §6.1) — fire every tick, no dropout fault
    // modeled for these yet (§9's fault list only covers GPS/IMU/drag).
    {
        const Eigen::Vector3d z_gyro = gyro_.measure(omega_true, dt);
        Eigen::Matrix<double, 3, 12> H_gyro = Eigen::Matrix<double, 3, 12>::Zero();
        H_gyro.block<3, 3>(0, 3).setIdentity();
        const Eigen::Matrix3d R_gyro = Eigen::Matrix3d::Identity() * (cfg_.gyro_sigma * cfg_.gyro_sigma);

        ekf_.H = H_gyro; ekf_.R = R_gyro; ekf_.update(z_gyro);
        ukf_.H = H_gyro; ukf_.R = R_gyro; ukf_.update(z_gyro);

        const Eigen::Matrix3d R_body_eci_true = q_true.conjugate().toRotationMatrix();
        const Eigen::Vector3d z_mag = mag_.measure_body(x_true_.head<3>(), jd_now, R_body_eci_true);
        const Eigen::Matrix3d R_mag = Eigen::Matrix3d::Identity() * (cfg_.mag_sigma * cfg_.mag_sigma);

        auto mag_update = [&](auto& filt) {
            const Eigen::Vector3d r_hat = filt.x.template segment<3>(6);
            const Eigen::Vector3d b_eci = sensors::MagnetometerSensor::field_eci(r_hat, jd_now);
            const Eigen::Vector3d z_pred = filt.q_ref.conjugate().toRotationMatrix() * b_eci;
            Eigen::Matrix<double, 3, 12> H_mag = Eigen::Matrix<double, 3, 12>::Zero();
            H_mag.block<3, 3>(0, 0) = math::skew(z_pred);
            filt.H = H_mag;
            filt.R = R_mag;
            filt.update(z_mag);
        };
        mag_update(ekf_);
        mag_update(ukf_);

        // MEKF reset (math.md §7.3): fold all of this tick's accumulated
        // delta_theta correction into q_ref once, after every update()
        // call above — not after each one individually.
        ekf_.reset_attitude_error();
        ukf_.reset_attitude_error();
    }

    for (int i = 0; i < 3; ++i) {
        frame.kf_pos[i]  = kf_.x(i);
        frame.kf_vel[i]  = kf_.x(i + 3);
        frame.ekf_pos[i] = ekf_.x(i + 6);
        frame.ekf_vel[i] = ekf_.x(i + 9);
        frame.ekf_omega[i] = ekf_.x(i + 3);
        frame.ukf_pos[i] = ukf_.x(i + 6);
        frame.ukf_vel[i] = ukf_.x(i + 9);
        frame.ukf_omega[i] = ukf_.x(i + 3);
    }
    for (int i = 0; i < 4; ++i) {
        frame.ekf_quat[i] = ekf_.q_ref.coeffs()(i);
        frame.ukf_quat[i] = ukf_.q_ref.coeffs()(i);
    }
    for (int i = 0; i < 6; ++i) {
        frame.kf_cov_diag[i]  = kf_.P(i, i);
    }
    for (int i = 0; i < 12; ++i) {
        frame.ekf_cov_diag[i] = ekf_.P(i, i);
        frame.ukf_cov_diag[i] = ukf_.P(i, i);
    }
    frame.active_fault = static_cast<uint8_t>(reported_fault);

    sim_time_.store(t_now + dt, std::memory_order_relaxed);
    ring_buffer_.push(frame);
}

namespace {
Simulation& global_simulation() {
    static Simulation sim;
    return sim;
}
}  // namespace

void init_scenario(const ScenarioCfg& cfg) { global_simulation().init_scenario(cfg); }
void start_simulation() { global_simulation().start(); }
void pause_simulation() { global_simulation().pause(); }
void reset_simulation() { global_simulation().reset(); }
uintptr_t get_ring_buffer_ptr() { return global_simulation().get_ring_buffer_ptr(); }
size_t get_ring_buffer_capacity() { return Simulation::get_ring_buffer_capacity(); }
void set_fault(const faults::FaultConfig& fault) { global_simulation().set_fault(fault); }
double get_sim_time() { return global_simulation().get_sim_time(); }
bool is_running() { return global_simulation().is_running(); }

void run_monte_carlo(size_t n_runs, int seed) { global_simulation().run_monte_carlo(n_runs, seed); }
const monte_carlo::MCStats& get_mc_results() { return global_simulation().get_mc_results(); }
size_t get_mc_n_runs() { return global_simulation().get_mc_n_runs(); }

}  // namespace orbitforge

#ifdef __EMSCRIPTEN__
#include <cstdio>

#include <emscripten/emscripten.h>

// The build exports ccall/cwrap (EXPORTED_RUNTIME_METHODS), and the JS side
// (wasm_types.ts) already commits to the ccall calling convention
// (OrbitForgeModule.ccall/cwrap, not embind value objects). These
// extern "C" + EMSCRIPTEN_KEEPALIVE exports are what ccall('init_scenario',
// ...) etc. resolve to — matching the TS bridge rather than embind's
// EMSCRIPTEN_BINDINGS macro. UNVERIFIED: no Emscripten toolchain installed
// in this dev environment — smoke-test these exports in CI before relying
// on them.
extern "C" {

EMSCRIPTEN_KEEPALIVE
void init_scenario(const char* tle_line1, const char* tle_line2, double gps_sigma,
                    double imu_sigma, int enable_j2, int enable_drag, int enable_srp,
                    double drag_coeff, double area_to_mass, double q_pos, double q_vel,
                    double sim_speed, int seed,
                    // Phase 5: 6DOF (CLAUDE.md §21) — appended after the
                    // Phase 1-4 params so existing positional ccall sites
                    // would only break if they don't pass these, not
                    // silently misread an earlier argument.
                    double inertia_x, double inertia_y, double inertia_z,
                    double gyro_sigma, double mag_sigma, double q_att, double q_omega,
                    double init_omega_x, double init_omega_y, double init_omega_z) {
    orbitforge::ScenarioCfg cfg;
    std::snprintf(cfg.tle_line1, sizeof(cfg.tle_line1), "%s", tle_line1);
    std::snprintf(cfg.tle_line2, sizeof(cfg.tle_line2), "%s", tle_line2);
    cfg.gps_sigma = gps_sigma;
    cfg.imu_sigma = imu_sigma;
    cfg.enable_j2 = enable_j2 != 0;
    cfg.enable_drag = enable_drag != 0;
    cfg.enable_srp = enable_srp != 0;
    cfg.drag_coeff = drag_coeff;
    cfg.area_to_mass = area_to_mass;
    cfg.q_pos = q_pos;
    cfg.q_vel = q_vel;
    cfg.sim_speed = sim_speed;
    cfg.seed = seed;
    cfg.inertia_x = inertia_x;
    cfg.inertia_y = inertia_y;
    cfg.inertia_z = inertia_z;
    cfg.gyro_sigma = gyro_sigma;
    cfg.mag_sigma = mag_sigma;
    cfg.q_att = q_att;
    cfg.q_omega = q_omega;
    cfg.init_omega_x = init_omega_x;
    cfg.init_omega_y = init_omega_y;
    cfg.init_omega_z = init_omega_z;
    orbitforge::init_scenario(cfg);
}

EMSCRIPTEN_KEEPALIVE
void start_simulation() { orbitforge::start_simulation(); }

EMSCRIPTEN_KEEPALIVE
void pause_simulation() { orbitforge::pause_simulation(); }

EMSCRIPTEN_KEEPALIVE
void reset_simulation() { orbitforge::reset_simulation(); }

EMSCRIPTEN_KEEPALIVE
uintptr_t get_ring_buffer_ptr() { return orbitforge::get_ring_buffer_ptr(); }

EMSCRIPTEN_KEEPALIVE
size_t get_ring_buffer_capacity() { return orbitforge::get_ring_buffer_capacity(); }

EMSCRIPTEN_KEEPALIVE
void set_fault(int fault_type, double onset_t, double duration, double magnitude) {
    orbitforge::faults::FaultConfig cfg;
    cfg.type = static_cast<orbitforge::faults::FaultType>(fault_type);
    cfg.onset_t = onset_t;
    cfg.duration = duration;
    cfg.magnitude = magnitude;
    orbitforge::set_fault(cfg);
}

EMSCRIPTEN_KEEPALIVE
double get_sim_time() { return orbitforge::get_sim_time(); }

EMSCRIPTEN_KEEPALIVE
int is_running() { return orbitforge::is_running() ? 1 : 0; }

// run_monte_carlo blocks the calling worker thread until the campaign
// completes — it does not return early and post a
// separate completion message; ccall('run_monte_carlo', ...) is a
// synchronous call on the JS side, same as the other void exports here.
// MCStats arrays are exposed the same way the ring buffer is (raw pointer
// + count, not a copy): the returned uintptr_t is a byte offset into WASM
// linear memory, valid until the next run_monte_carlo() call reallocates
// the underlying std::vector.
EMSCRIPTEN_KEEPALIVE
void run_monte_carlo(int n_runs, int seed) {
    orbitforge::run_monte_carlo(n_runs > 0 ? static_cast<size_t>(n_runs) : 0, seed);
}

EMSCRIPTEN_KEEPALIVE
int get_mc_n_steps() { return static_cast<int>(orbitforge::get_mc_results().rms_pos.size()); }

EMSCRIPTEN_KEEPALIVE
int get_mc_n_runs() { return static_cast<int>(orbitforge::get_mc_n_runs()); }

EMSCRIPTEN_KEEPALIVE
uintptr_t get_mc_rms_pos_ptr() {
    return reinterpret_cast<uintptr_t>(orbitforge::get_mc_results().rms_pos.data());
}

EMSCRIPTEN_KEEPALIVE
uintptr_t get_mc_rms_vel_ptr() {
    return reinterpret_cast<uintptr_t>(orbitforge::get_mc_results().rms_vel.data());
}

EMSCRIPTEN_KEEPALIVE
uintptr_t get_mc_nees_ptr() {
    return reinterpret_cast<uintptr_t>(orbitforge::get_mc_results().nees.data());
}

EMSCRIPTEN_KEEPALIVE
uintptr_t get_mc_nis_ptr() {
    return reinterpret_cast<uintptr_t>(orbitforge::get_mc_results().nis.data());
}

EMSCRIPTEN_KEEPALIVE
uintptr_t get_mc_final_pos_err_ptr() {
    return reinterpret_cast<uintptr_t>(orbitforge::get_mc_results().final_pos_err.data());
}

EMSCRIPTEN_KEEPALIVE
double get_mc_nees_lower() {
    return orbitforge::monte_carlo::nees_bounds(orbitforge::get_mc_n_runs()).lower;
}

EMSCRIPTEN_KEEPALIVE
double get_mc_nees_upper() {
    return orbitforge::monte_carlo::nees_bounds(orbitforge::get_mc_n_runs()).upper;
}

EMSCRIPTEN_KEEPALIVE
double get_mc_nis_lower() {
    return orbitforge::monte_carlo::nis_bounds(orbitforge::get_mc_n_runs()).lower;
}

EMSCRIPTEN_KEEPALIVE
double get_mc_nis_upper() {
    return orbitforge::monte_carlo::nis_bounds(orbitforge::get_mc_n_runs()).upper;
}

}  // extern "C"
#endif  // __EMSCRIPTEN__
