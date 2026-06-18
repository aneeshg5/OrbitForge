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

}  // namespace

Simulation::Simulation() = default;

Simulation::~Simulation() {
    running_.store(false, std::memory_order_relaxed);
    if (worker_thread_.joinable()) worker_thread_.join();
}

void Simulation::init_filters() {
    seed_filter(kf_, x_true_, cfg_.q_pos, cfg_.q_vel, cfg_.gps_sigma);
    seed_filter(ekf_, x_true_, cfg_.q_pos, cfg_.q_vel, cfg_.gps_sigma);
    seed_filter(ukf_, x_true_, cfg_.q_pos, cfg_.q_vel, cfg_.gps_sigma);

    ekf_.perturb_cfg = perturb_nominal_;
    ekf_.julian_date = epoch_jd_;
    ukf_.perturb_cfg = perturb_nominal_;
    ukf_.julian_date = epoch_jd_;
    ukf_.S = ukf_.P.llt().matrixL();
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

    kf_.predict(dt);
    ekf_.predict(dt);
    ukf_.predict(dt);

    StateFrame frame;
    frame.sim_time = t_now + dt;
    for (int i = 0; i < 3; ++i) {
        frame.true_pos[i] = x_true_(i);
        frame.true_vel[i] = x_true_(i + 3);
    }

    if (!gps_dropout_active) {
        const Eigen::Matrix3d R_ecef_eci = sensors::GpsSensor::R_ecef_eci(jd_now);
        Eigen::Vector3d z = gps_.measure(x_true_.head<3>(), jd_now);
        if (gps_spike_offset != 0.0) {
            z += Eigen::Vector3d(gps_spike_offset, 0.0, 0.0);
        }

        Eigen::Matrix<double, 3, 6> H;
        H << R_ecef_eci, Eigen::Matrix3d::Zero();
        kf_.H = H;
        ekf_.H = H;
        ukf_.H = H;

        auto nis_of = [&z](const filters::FilterBase<6, 3>& f, const Eigen::Matrix<double, 3, 6>& h) {
            const Eigen::Vector3d nu = z - h * f.x;
            const Eigen::Matrix3d S = h * f.P * h.transpose() + f.R;
            return nu.dot(S.ldlt().solve(nu));
        };
        frame.kf_nis  = nis_of(kf_, H);
        frame.ekf_nis = nis_of(ekf_, H);
        frame.ukf_nis = nis_of(ukf_, H);

        kf_.update(z);
        ekf_.update(z);
        ukf_.update(z);
    }

    for (int i = 0; i < 3; ++i) {
        frame.kf_pos[i]  = kf_.x(i);
        frame.kf_vel[i]  = kf_.x(i + 3);
        frame.ekf_pos[i] = ekf_.x(i);
        frame.ekf_vel[i] = ekf_.x(i + 3);
        frame.ukf_pos[i] = ukf_.x(i);
        frame.ukf_vel[i] = ukf_.x(i + 3);
    }
    for (int i = 0; i < 6; ++i) {
        frame.kf_cov_diag[i]  = kf_.P(i, i);
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
                    double sim_speed, int seed) {
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
