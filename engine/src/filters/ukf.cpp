#include "filters/ukf.hpp"
#include "integrators/rk4.hpp"
#include "dynamics/eom.hpp"
#include "constants.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace orbitforge::filters {

namespace {

// UKF scaled parameters: alpha=1e-3, kappa=0, beta=2
// lambda = alpha^2*(n+kappa) - n = (1e-3)^2*6 - 6 ≈ -5.999994
// n+lambda = alpha^2*(n+kappa) ≈ 6e-6  (sigma points very close to mean)
// W_m0 ≈ -999999,  W_mi = W_ci ≈ 83333,  W_c0 ≈ -999996
// (math.md §4)
constexpr int    k_n   = 6;   // state dim
constexpr int    k_m   = 3;   // measurement dim
constexpr int    k_ns  = 2 * k_n + 1;  // 13 sigma points

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
    : julian_date(orbitforge::k_j2000_jd)
{
    x.setZero();
    P.setIdentity();
    Q.setZero();
    R.setZero();
    S.setIdentity();   // S = I₆ = chol(P=I₆)
    H.setZero();
    H.block<3, 3>(0, 0).setIdentity();
}

void UnscentedKalmanFilter::predict(double dt)
{
    double gamma, W_m0, W_mi, W_c0, W_ci;
    ukf_params(gamma, W_m0, W_mi, W_c0, W_ci);

    // === 1. Sigma points from current x and S ===
    // chi_0 = x;  chi_i = x + gamma·S[:,i-1];  chi_{i+n} = x - gamma·S[:,i-1]
    Eigen::Matrix<double, k_n, k_ns> chi;
    chi.col(0) = x;
    for (int i = 0; i < k_n; ++i) {
        chi.col(i + 1)       = x + gamma * S.col(i);
        chi.col(i + k_n + 1) = x - gamma * S.col(i);
    }

    // === 2. Propagate each sigma point through RK4 ===
    const double jd  = julian_date;
    const auto&  cfg = perturb_cfg;

    auto f_dyn = [&](double, const Eigen::Matrix<double, k_n, 1>& s) {
        Eigen::Matrix<double, k_n, 1> ds;
        ds.head<3>() = s.tail<3>();
        ds.tail<3>() = dynamics::compute_acceleration(s.head<3>(), s.tail<3>(), jd, cfg);
        return ds;
    };

    Eigen::Matrix<double, k_n, k_ns> chi_star;
    for (int i = 0; i < k_ns; ++i) {
        // rk4_step is templated on State; evaluate col(i) to a concrete vector
        // so the deduced type is Matrix<double,6,1>, not a Block expression.
        const Eigen::Matrix<double, k_n, 1> chi_i = chi.col(i);
        chi_star.col(i) = rk4_step(chi_i, 0.0, dt, f_dyn);
    }

    // === 3. Predicted mean ===
    Eigen::Matrix<double, k_n, 1> x_pred = W_m0 * chi_star.col(0);
    for (int i = 1; i < k_ns; ++i) x_pred += W_mi * chi_star.col(i);

    // === 4. SR covariance via QR of [sqrt(W_ci)·deviations | S_q] ===
    // A^T has shape (2n+n)×n = 18×6:
    //   rows 0..11:  sqrt(W_ci)·(chi_star_i - x_pred)^T  for i=1..12
    //   rows 12..17: S_q   (lower-triangular Cholesky of Q)
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
    // W_c0 < 0 for alpha=1e-3, so this is a downdate.
    // d0 ≈ 0 for smooth dynamics (chi_star_0 ≈ x_pred), so the downdate is tiny.
    const Eigen::Matrix<double, k_n, 1> d0 = chi_star.col(0) - x_pred;
    if (W_c0 < 0.0) {
        chol_rank1_update(S, std::sqrt(-W_c0) * d0, -1.0);
    } else {
        chol_rank1_update(S, std::sqrt( W_c0) * d0, +1.0);
    }

    x = x_pred;
    julian_date += dt / orbitforge::k_sec_per_day;
    P = S * S.transpose();
}

void UnscentedKalmanFilter::update(const Eigen::Matrix<double, 3, 1>& z)
{
    // For linear H (GPS position-only), the sigma-point weighted sums reduce to:
    //   P_xy = P·Hᵀ  and  P_yy = H·P·Hᵀ + R
    // so the Kalman gain and state update are identical to the EKF.
    const Eigen::Matrix<double, 3, 3> S_innov = H * P * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S_innov.inverse();

    x = x + K * (z - H * x);

    // SR covariance update: P_new = P - K·S_innov·Kᵀ = S·Sᵀ - U·Uᵀ
    // where U = K·chol(S_innov). Apply m=3 rank-1 Cholesky downdates.
    Eigen::LLT<Eigen::Matrix<double, 3, 3>> llt_innov(S_innov);
    const Eigen::Matrix<double, 3, 3> S_innov_L = llt_innov.matrixL();
    const Eigen::Matrix<double, 6, 3> U = K * S_innov_L;
    for (int j = 0; j < k_m; ++j) {
        chol_rank1_update(S, U.col(j), -1.0);
    }
    P = S * S.transpose();
}

} // namespace orbitforge::filters
