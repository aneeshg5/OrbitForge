#pragma once

#include "filters/filter_base.hpp"
#include "dynamics/eom.hpp"

namespace orbitforge::filters {

// Extended Kalman Filter for 6-DOF orbital state with 3-axis GPS measurements.
// State propagated nonlinearly via RK4. Covariance propagated via first-order
// Jacobian Φ = I + F·dt where F includes gravity + J2 (math.md §3.1–3.2).
// Drag velocity coupling is intentionally omitted from F (conservative-force approx).
class ExtendedKalmanFilter : public FilterBase<6, 3> {
public:
    Eigen::Matrix<double, 3, 6>  H;            // measurement Jacobian (caller sets per tick)
    dynamics::PerturbationConfig perturb_cfg;  // which forces to include in state propagation
    double                       julian_date;  // current epoch; advanced by predict()

    // Initialises H = [I₃ | 0₃], julian_date = J2000.
    ExtendedKalmanFilter();

    // Predict: propagates x via RK4, propagates P via Φ = I + F·dt.
    void predict(double dt) override;
    // Update: standard Kalman equations with 3×3 S solve.
    void update(const Eigen::Matrix<double, 3, 1>& z) override;
};

} // namespace orbitforge::filters
