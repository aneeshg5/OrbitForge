// math.md §8.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <cmath>

#include "filters/ekf.hpp"
#include "filters/ukf.hpp"
#include "integrators/rk4.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

using namespace orbitforge;
using namespace orbitforge::filters;
using namespace orbitforge::dynamics;

namespace {

constexpr double k_nees_lower  = 5.35;
constexpr double k_nees_upper  = 6.69;
constexpr double k_min_in_frac = 0.90;

static constexpr double k_r0 = k_re + 408e3;
static inline double iss_v0() { return std::sqrt(k_mu / k_r0); }

// math.md §7.3.
template <typename Filter, int Off, typename MakeFilter>
std::vector<double> run_mc_nees(int N, int n_steps, double dt,
                                 double sigma_gps,
                                 double q_pos, double q_vel,
                                 MakeFilter&& make_filter)
{
    PerturbationConfig two_body;
    two_body.enable_j2   = false;
    two_body.enable_drag = false;
    two_body.enable_srp  = false;

    auto f_dyn = [&two_body](double, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = compute_acceleration(s.head<3>(), s.tail<3>(), k_j2000_jd, two_body);
        return ds;
    };

    const double v0 = iss_v0();

    std::vector<double> nees_sum(n_steps, 0.0);
    std::vector<int>    nees_cnt(n_steps, 0);

    for (int run = 0; run < N; ++run) {
        std::mt19937 rng(static_cast<unsigned>(run) * 31337u + 7u);
        std::normal_distribution<double> pos_ic(0.0, 100.0);
        std::normal_distribution<double> vel_ic(0.0, 1.0);
        std::normal_distribution<double> gps_noise(0.0, sigma_gps);
        std::normal_distribution<double> proc_pos(0.0, q_pos);
        std::normal_distribution<double> proc_vel(0.0, q_vel);

        Eigen::Matrix<double, 6, 1> x_true;
        x_true << k_r0, 0.0, 0.0, 0.0, v0, 0.0;

        Filter flt = make_filter();
        flt.x.template segment<3>(Off)     = Eigen::Vector3d(k_r0 + pos_ic(rng), pos_ic(rng), pos_ic(rng));
        flt.x.template segment<3>(Off + 3) = Eigen::Vector3d(vel_ic(rng), v0 + vel_ic(rng), vel_ic(rng));

        for (int step = 0; step < n_steps; ++step) {
            x_true = rk4_step(x_true, 0.0, dt, f_dyn);
            x_true[0] += proc_pos(rng);  x_true[1] += proc_pos(rng);  x_true[2] += proc_pos(rng);
            x_true[3] += proc_vel(rng);  x_true[4] += proc_vel(rng);  x_true[5] += proc_vel(rng);

            flt.predict(dt);

            Eigen::Vector3d z = x_true.head<3>() +
                Eigen::Vector3d(gps_noise(rng), gps_noise(rng), gps_noise(rng));
            flt.update(z);

            Eigen::Matrix<double, 6, 1> err;
            err.template head<3>() = x_true.template head<3>() - flt.x.template segment<3>(Off);
            err.template tail<3>() = x_true.template tail<3>() - flt.x.template segment<3>(Off + 3);
            const Eigen::Matrix<double, 6, 6> P_orbit = flt.P.template block<6, 6>(Off, Off);
            Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(P_orbit);
            if (llt.info() == Eigen::Success) {
                nees_sum[step] += err.dot(llt.solve(err));
                nees_cnt[step]++;
            }
        }
    }

    std::vector<double> nees_avg(n_steps, 0.0);
    for (int step = 0; step < n_steps; ++step) {
        if (nees_cnt[step] > 0)
            nees_avg[step] = nees_sum[step] / static_cast<double>(nees_cnt[step]);
    }
    return nees_avg;
}

static void check_nees_consistency(const std::vector<double>& nees, const char* label)
{
    int in_bounds = 0;
    for (double v : nees)
        if (v >= k_nees_lower && v <= k_nees_upper) ++in_bounds;

    const double frac = static_cast<double>(in_bounds) / static_cast<double>(nees.size());
    EXPECT_GE(frac, k_min_in_frac)
        << label << " NEES fraction in [" << k_nees_lower << ", " << k_nees_upper
        << "]: " << frac << " (need >= " << k_min_in_frac << ")";
}

}

TEST(EKFConsistency, NEESWithinBounds) {
    constexpr double q_pos = 1.0;
    constexpr double q_vel = 0.01;

    const auto nees = run_mc_nees<ExtendedKalmanFilter, 6>(
        100, 500, 10.0, 10.0, q_pos, q_vel,
        [=] {
            ExtendedKalmanFilter ekf;
            ekf.P.setZero();
            for (int i = 6; i < 9; ++i) ekf.P(i, i) = 100.0 * 100.0;
            for (int i = 9; i < 12; ++i) ekf.P(i, i) = 1.0;
            ekf.Q.setZero();
            for (int i = 6; i < 9; ++i) ekf.Q(i, i) = q_pos * q_pos;
            for (int i = 9; i < 12; ++i) ekf.Q(i, i) = q_vel * q_vel;
            ekf.R.setZero();
            for (int i = 0; i < 3; ++i) ekf.R(i, i) = 10.0 * 10.0;
            ekf.perturb_cfg.enable_j2   = false;
            ekf.perturb_cfg.enable_drag = false;
            ekf.perturb_cfg.enable_srp  = false;
            return ekf;
        });
    check_nees_consistency(nees, "EKF");
}

TEST(UKFConsistency, NEESWithinBounds) {
    constexpr double q_pos = 1.0;
    constexpr double q_vel = 0.01;

    const auto nees = run_mc_nees<UnscentedKalmanFilter, 6>(
        100, 500, 10.0, 10.0, q_pos, q_vel,
        [=] {
            UnscentedKalmanFilter ukf;
            ukf.P.setIdentity();
            for (int i = 6; i < 9; ++i) ukf.P(i, i) = 100.0 * 100.0;
            for (int i = 9; i < 12; ++i) ukf.P(i, i) = 1.0;
            {
                Eigen::LLT<Eigen::Matrix<double, 12, 12>> llt(ukf.P);
                ukf.S = llt.matrixL();
            }
            ukf.Q.setZero();
            ukf.Q.diagonal().setConstant(1e-12);
            for (int i = 6; i < 9; ++i) ukf.Q(i, i) = q_pos * q_pos;
            for (int i = 9; i < 12; ++i) ukf.Q(i, i) = q_vel * q_vel;
            ukf.R.setZero();
            for (int i = 0; i < 3; ++i) ukf.R(i, i) = 10.0 * 10.0;
            ukf.perturb_cfg.enable_j2   = false;
            ukf.perturb_cfg.enable_drag = false;
            ukf.perturb_cfg.enable_srp  = false;
            return ukf;
        });
    check_nees_consistency(nees, "UKF");
}
