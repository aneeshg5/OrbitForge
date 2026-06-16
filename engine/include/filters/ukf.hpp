#pragma once

#include "filters/filter_base.hpp"
#include "dynamics/eom.hpp"

namespace orbitforge::filters {

// Unscented Kalman Filter — square-root form (SR-UKF).
// Maintains S (lower triangular Cholesky of P) directly; P = S·Sᵀ is never
// accumulated in the hot path. Predict uses QR to form S⁻; update uses
// rank-1 Cholesky downdates (cholupdate inline, no LAPACK). See math.md §4.
class UnscentedKalmanFilter : public FilterBase<6, 3> {
public:
    Eigen::Matrix<double, 6, 6>  S;            // lower triangular: P = S·Sᵀ
    Eigen::Matrix<double, 3, 6>  H;            // measurement Jacobian (caller sets per tick)
    dynamics::PerturbationConfig perturb_cfg;
    double                       julian_date;

    // Initialises H = [I₃ | 0₃], S = I₆, julian_date = J2000.
    UnscentedKalmanFilter();

    // Predict: propagates 13 sigma points via RK4, reconstructs S⁻ via QR + rank-1 downdate.
    void predict(double dt) override;
    // Update: standard Kalman equations with Joseph-form P refresh and S = chol(P_new).
    void update(const Eigen::Matrix<double, 3, 1>& z) override;
};

} // namespace orbitforge::filters
