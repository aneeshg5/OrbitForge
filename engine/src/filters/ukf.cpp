#include "filters/ukf.hpp"
#include "integrators/rk4.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace orbitforge::filters {

namespace {

// math.md §4.1.
constexpr int    k_n   = 12;
constexpr int    k_m   = 3;
constexpr int    k_ns  = 2 * k_n + 1;

inline void ukf_params(double& gamma, double& W_m0, double& W_mi,
                       double& W_c0, double& W_ci)
{
    constexpr double alpha  = 1e-3;
    constexpr double kappa  = 0.0;
    constexpr double beta   = 2.0;
    const double     lambda = alpha*alpha*(k_n + kappa) - k_n;
    const double     nl     = k_n + lambda;
    gamma = std::sqrt(nl);
    W_m0  = lambda / nl;
    W_mi  = 0.5 / nl;
    W_c0  = W_m0 + (1.0 - alpha*alpha + beta);
    W_ci  = W_mi;
}

bool chol_rank1_update(Eigen::Matrix<double, k_n, k_n>& L,
                       Eigen::Matrix<double, k_n, 1>    v,
                       double                            sign)
{
    for (int k = 0; k < k_n; ++k) {
        const double L_kk = L(k, k);
        const double v_k  = v[k];
        const double rr   = L_kk*L_kk + sign*v_k*v_k;
        if (rr <= 0.0) return false;
        const double r = std::sqrt(rr);
        const double c = L_kk / r;
        const double s = v_k  / r;
        L(k, k) = r;
        for (int i = k + 1; i < k_n; ++i) {
            const double L_ik = L(i, k);
            const double v_i  = v[i];
            L(i, k) = c * L_ik + sign * s * v_i;
            v[i]    = c * v_i  - s * L_ik;
        }
    }
    return true;
}

template <int M, int N>
Eigen::Matrix<double, N, N>
chol_from_qr(const Eigen::HouseholderQR<Eigen::Matrix<double, M, N>>& qr)
{
    const auto& mat = qr.matrixQR();

    Eigen::Matrix<double, N, N> R;
    R.setZero();
    for (int r = 0; r < N; ++r)
        for (int c = r; c < N; ++c)
            R(r, c) = mat(r, c);

    Eigen::Matrix<double, N, N> S = R.transpose();

    for (int i = 0; i < N; ++i) {
        if (S(i, i) < 0.0) {
            for (int j = i; j < N; ++j) S(j, i) = -S(j, i);
        }
    }
    return S;
}

}

UnscentedKalmanFilter::UnscentedKalmanFilter()
    : julian_date(orbitforge::k_j2000_jd),
      q_ref(math::Quat::Identity())
{
    x.setZero();
    P.setIdentity();
    Q.setZero();
    R.setZero();
    S.setIdentity();
    H.setZero();
    H.block<3, 3>(0, 6).setIdentity();
}

void UnscentedKalmanFilter::predict(double dt)
{
    double gamma, W_m0, W_mi, W_c0, W_ci;
    ukf_params(gamma, W_m0, W_mi, W_c0, W_ci);

    Eigen::Matrix<double, k_n, k_ns> chi;
    chi.col(0) = x;
    for (int i = 0; i < k_n; ++i) {
        chi.col(i + 1)       = x + gamma * S.col(i);
        chi.col(i + k_n + 1) = x - gamma * S.col(i);
    }

    const double jd  = julian_date;
    const auto&  cfg = perturb_cfg;
    const dynamics::InertiaTensor inertia_local = inertia;
    const math::Quat q_ref_local = q_ref;

    auto orbital_dyn = [&jd, &cfg](double , const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), jd, cfg);
        return ds;
    };
    auto att_dyn = [&inertia_local](double , const dynamics::AttitudeState& s) {
        return dynamics::attitude_derivative(s, inertia_local);
    };

    std::array<math::Quat, k_ns> q_star;
    Eigen::Matrix<double, k_n, k_ns> chi_star;

    for (int i = 0; i < k_ns; ++i) {
        const Eigen::Vector3d delta_theta_i = chi.col(i).head<3>();
        const Eigen::Vector3d omega_i       = chi.col(i).segment<3>(3);
        const Eigen::Matrix<double, 6, 1> rv_i = chi.col(i).tail<6>();

        dynamics::AttitudeState att_i;
        att_i.head<4>() = (q_ref_local * math::quat_exp(delta_theta_i)).normalized().coeffs();
        att_i.tail<3>() = omega_i;
        att_i = rk4_step(att_i, 0.0, dt, att_dyn);

        q_star[i] = math::Quat(att_i[3], att_i[0], att_i[1], att_i[2]).normalized();
        chi_star.col(i).segment<3>(3) = att_i.tail<3>();
        chi_star.col(i).tail<6>() = rk4_step(rv_i, 0.0, dt, orbital_dyn);
    }

    const math::Quat q_ref_new = q_star[0];
    for (int i = 0; i < k_ns; ++i) {
        const math::Quat delta_q = (q_ref_new.conjugate() * q_star[i]).normalized();
        chi_star.col(i).head<3>() = math::quat_log(delta_q);
    }

    Eigen::Matrix<double, k_n, 1> x_pred = W_m0 * chi_star.col(0);
    for (int i = 1; i < k_ns; ++i) x_pred += W_mi * chi_star.col(i);

    Eigen::LLT<Eigen::Matrix<double, k_n, k_n>> llt_Q(Q);
    const Eigen::Matrix<double, k_n, k_n> S_q = llt_Q.matrixL();

    const double sqrt_Wci = std::sqrt(W_ci);
    Eigen::Matrix<double, 2*k_n + k_n, k_n> AT;
    for (int i = 1; i < k_ns; ++i) {
        AT.row(i - 1) = (sqrt_Wci * (chi_star.col(i) - x_pred)).transpose();
    }
    AT.block<k_n, k_n>(2 * k_n, 0) = S_q;

    Eigen::HouseholderQR<Eigen::Matrix<double, 2*k_n + k_n, k_n>> qr(AT);
    S = chol_from_qr(qr);

    const Eigen::Matrix<double, k_n, 1> d0 = chi_star.col(0) - x_pred;
    if (W_c0 < 0.0) {
        chol_rank1_update(S, std::sqrt(-W_c0) * d0, -1.0);
    } else {
        chol_rank1_update(S, std::sqrt( W_c0) * d0, +1.0);
    }

    x = x_pred;
    q_ref = q_ref_new;
    julian_date += dt / orbitforge::k_sec_per_day;
    P = S * S.transpose();
}

void UnscentedKalmanFilter::update(const Eigen::Matrix<double, 3, 1>& z)
{
    const Eigen::Matrix<double, 3, 3>  S_innov = H * P * H.transpose() + R;
    const Eigen::Matrix<double, 12, 3> K = P * H.transpose() * S_innov.inverse();

    x = x + K * (z - H * x);

    Eigen::LLT<Eigen::Matrix<double, 3, 3>> llt_innov(S_innov);
    const Eigen::Matrix<double, 3, 3>  S_innov_L = llt_innov.matrixL();
    const Eigen::Matrix<double, 12, 3> U = K * S_innov_L;
    for (int j = 0; j < k_m; ++j) {
        chol_rank1_update(S, U.col(j), -1.0);
    }
    P = S * S.transpose();
}

void UnscentedKalmanFilter::reset_attitude_error() {
    const Eigen::Vector3d delta_theta = x.head<3>();
    q_ref = (q_ref * math::quat_exp(delta_theta)).normalized();
    x.head<3>().setZero();
}

}
