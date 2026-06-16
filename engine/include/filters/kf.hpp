#pragma once

#include "filters/filter_base.hpp"

namespace orbitforge::filters {

// Linear Kalman Filter for 6-DOF orbital state with 3-axis GPS measurements.
// Uses a first-order two-body gravity Jacobian for the state transition.
// Intentionally diverges on curved orbits — linearisation error accumulates
// visibly over time. See math.md §3.1.
class KalmanFilter : public FilterBase<6, 3> {
public:
    Eigen::Matrix<double, 3, 6> H;  // measurement matrix (caller sets per tick if ECEF)

    // Initialises H = [I₃ | 0₃] (GPS measures ECI position directly).
    KalmanFilter();

    // Predict step: Φ = I + F·dt, F from two-body gravity Jacobian at x̂.
    void predict(double dt) override;
    // Update step: standard Kalman equations with 3×3 S solve.
    void update(const Eigen::Matrix<double, 3, 1>& z) override;
};

} // namespace orbitforge::filters
