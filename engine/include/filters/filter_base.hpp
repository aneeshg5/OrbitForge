#pragma once

#include <Eigen/Dense>

namespace orbitforge::filters {

// Abstract base for all orbital state estimators.
// N_STATE: state vector dimension (6 for [r; v]).
// N_MEAS:  measurement vector dimension (3 for GPS position).
template <int N_STATE, int N_MEAS>
class FilterBase {
public:
    // Propagate state estimate forward by dt seconds (no measurement).
    virtual void predict(double dt) = 0;
    // Incorporate measurement z into the state estimate.
    virtual void update(const Eigen::Matrix<double, N_MEAS, 1>& z) = 0;
    virtual ~FilterBase() = default;

    Eigen::Matrix<double, N_STATE, 1>       x;  // state estimate [r; v] ECI (m, m/s)
    Eigen::Matrix<double, N_STATE, N_STATE> P;  // error covariance
    Eigen::Matrix<double, N_STATE, N_STATE> Q;  // process noise covariance
    Eigen::Matrix<double, N_MEAS,  N_MEAS>  R;  // measurement noise covariance
};

} // namespace orbitforge::filters
