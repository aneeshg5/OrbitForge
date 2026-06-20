#include "filters/ukf.hpp"
#include "integrators/rk4.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace orbitforge::filters {

namespace {

// UKF scaled parameters: alpha=1e-3, kappa=0, beta=2, n=12 (Phase 5; was 6).
// lambda = alpha^2*(n+kappa) - n = (1e-3)^2*12 - 12 ≈ -11.999988
// (math.md §4.1's formulas, just n=12 now — weight values themselves
// change with n, so the old §4.1 worked example no longer applies verbatim)
constexpr int    k_n   = 12;  // state dim
constexpr int    k_m   = 3;   // measurement dim
constexpr int    k_ns  = 2 * k_n + 1;  // 25 sigma points

inline void ukf_params(double& gamma, double& W_m0, double& W_mi,
                       double& W_c0, double& W_ci)
{
    constexpr double alpha  = 1e-3;
    constexpr double kappa  = 0.0;
    constexpr double beta   = 2.0;
    const double     lambda = alpha*alpha*(k_n + kappa) - k_n;
    const double     nl     = k_n + lambda;  // n+lambda
    gamma = std::sqrt(nl);
    W_m0  = lambda / nl;
    W_mi  = 0.5 / nl;
    W_c0  = W_m0 + (1.0 - alpha*alpha + beta);
    W_ci  = W_mi;
}

// Rank-1 Cholesky update/downdate (Givens rotation, no LAPACK).
// sign = +1: L_new·L_newᵀ = L·Lᵀ + v·vᵀ
// sign = -1: L_new·L_newᵀ = L·Lᵀ - v·vᵀ  (requires result remains P.D.)
// Returns false if a downdate would destroy positive-definiteness.
bool chol_rank1_update(Eigen::Matrix<double, k_n, k_n>& L,
                       Eigen::Matrix<double, k_n, 1>    v,   // by value; modified internally
                       double                            sign)
{
    for (int k = 0; k < k_n; ++k) {
        const double L_kk = L(k, k);
        const double v_k  = v[k];
        const double rr   = L_kk*L_kk + sign*v_k*v_k;
        if (rr <= 0.0) return false;  // downdate failed: not P.D.
        const double r = std::sqrt(rr);
        const double c = L_kk / r;   // Givens cosine
        const double s = v_k  / r;   // Givens sine
        L(k, k) = r;
        for (int i = k + 1; i < k_n; ++i) {
            const double L_ik = L(i, k);  // save before overwrite
            const double v_i  = v[i];
            L(i, k) = c * L_ik + sign * s * v_i;
            v[i]    = c * v_i  - s * L_ik;
        }
    }
    return true;
}

// Extract the lower-triangular Cholesky factor from a HouseholderQR object.
// For QR of A^T (M×N), chol(A·Aᵀ) = Rᵀ where R is the N×N upper-triangular factor.
// Flips column signs to guarantee positive diagonal (QR can yield negative R(i,i)).
template <int M, int N>
Eigen::Matrix<double, N, N>
chol_from_qr(const Eigen::HouseholderQR<Eigen::Matrix<double, M, N>>& qr)
{
    const auto& mat = qr.matrixQR();

    // Extract upper-triangular R (upper N×N of packed QR storage)
    Eigen::Matrix<double, N, N> R;
    R.setZero();
    for (int r = 0; r < N; ++r)
        for (int c = r; c < N; ++c)
            R(r, c) = mat(r, c);

    // S = Rᵀ is lower-triangular Cholesky of A·Aᵀ
    Eigen::Matrix<double, N, N> S = R.transpose();

    // Ensure positive diagonal: flip column i if S(i,i) < 0 (preserves S·Sᵀ)
    for (int i = 0; i < N; ++i) {
        if (S(i, i) < 0.0) {
            for (int j = i; j < N; ++j) S(j, i) = -S(j, i);
        }
    }
    return S;
}

} // namespace

UnscentedKalmanFilter::UnscentedKalmanFilter()
    : julian_date(orbitforge::k_j2000_jd),
      q_ref(math::Quat::Identity())
{
    x.setZero();
    P.setIdentity();
    Q.setZero();
    R.setZero();
    S.setIdentity();   // S = I₁₂ = chol(P=I₁₂)
    H.setZero();
    H.block<3, 3>(0, 6).setIdentity();  // GPS-shaped default: position block, columns 6-8
}

void UnscentedKalmanFilter::predict(double dt)
{
    double gamma, W_m0, W_mi, W_c0, W_ci;
    ukf_params(gamma, W_m0, W_mi, W_c0, W_ci);

    // === 1. Sigma points from current x and S ===
    Eigen::Matrix<double, k_n, k_ns> chi;
    chi.col(0) = x;
    for (int i = 0; i < k_n; ++i) {
        chi.col(i + 1)       = x + gamma * S.col(i);
        chi.col(i + k_n + 1) = x - gamma * S.col(i);
    }

    // === 2. Propagate each sigma point ===
    // Attitude block (delta_theta, omega): each sigma point's delta_theta_i
    // is reified into an actual perturbed quaternion q_i = q_ref ⊗
    // quat_exp(delta_theta_i), propagated through the SAME nonlinear rigid-
    // body dynamics as the true trajectory (no Jacobian — the whole point
    // of UKF), then converted back to a delta_theta relative to the newly-
    // propagated reference q_ref_new (taken from sigma point 0, whose
    // delta_theta is exactly 0 since chi.col(0) == x exactly — no
    // perturbation — making its propagated q identical to what
    // ExtendedKalmanFilter::predict() computes for q_ref alone).
    // Orbital block (r,v): unchanged from Phase 1, fully decoupled from
    // the attitude block (Euler's equation doesn't depend on attitude;
    // orbital EOM doesn't depend on omega/attitude either).
    const double jd  = julian_date;
    const auto&  cfg = perturb_cfg;
    const dynamics::InertiaTensor inertia_local = inertia;
    const math::Quat q_ref_local = q_ref;

    auto orbital_dyn = [&jd, &cfg](double /*t*/, const Eigen::Matrix<double, 6, 1>& s) {
        Eigen::Matrix<double, 6, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), jd, cfg);
        return ds;
    };
    auto att_dyn = [&inertia_local](double /*t*/, const dynamics::AttitudeState& s) {
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

    // Sigma point 0 has delta_theta == 0 exactly, so q_star[0] IS the new q_ref.
    const math::Quat q_ref_new = q_star[0];
    for (int i = 0; i < k_ns; ++i) {
        const math::Quat delta_q = (q_ref_new.conjugate() * q_star[i]).normalized();
        chi_star.col(i).head<3>() = math::quat_log(delta_q);
    }

    // === 3. Predicted mean ===
    Eigen::Matrix<double, k_n, 1> x_pred = W_m0 * chi_star.col(0);
    for (int i = 1; i < k_ns; ++i) x_pred += W_mi * chi_star.col(i);

    // === 4. SR covariance via QR of [sqrt(W_ci)·deviations | S_q] ===
    Eigen::LLT<Eigen::Matrix<double, k_n, k_n>> llt_Q(Q);
    const Eigen::Matrix<double, k_n, k_n> S_q = llt_Q.matrixL();

    const double sqrt_Wci = std::sqrt(W_ci);  // W_ci > 0 always
    Eigen::Matrix<double, 2*k_n + k_n, k_n> AT;
    for (int i = 1; i < k_ns; ++i) {
        AT.row(i - 1) = (sqrt_Wci * (chi_star.col(i) - x_pred)).transpose();
    }
    AT.block<k_n, k_n>(2 * k_n, 0) = S_q;

    Eigen::HouseholderQR<Eigen::Matrix<double, 2*k_n + k_n, k_n>> qr(AT);
    S = chol_from_qr(qr);

    // === 5. Rank-1 update/downdate for i=0 sigma point ===
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
    // For linear H, the sigma-point weighted sums reduce to standard
    // Kalman equations, exactly as Phase 1 (the measurement models added
    // in Phase 5 — gyro, magnetometer — are also linearized the same way
    // here as in EKF; UKF's nonlinearity advantage is entirely in the
    // predict step's attitude propagation, not in these linear-H updates).
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

} // namespace orbitforge::filters
