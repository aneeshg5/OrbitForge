#pragma once

#include <Eigen/Dense>

namespace orbitforge::filters {

template <int N_STATE, int N_MEAS>
class FilterBase {
public:
    virtual void predict(double dt) = 0;
    virtual void update(const Eigen::Matrix<double, N_MEAS, 1>& z) = 0;
    virtual ~FilterBase() = default;

    Eigen::Matrix<double, N_STATE, 1>       x;
    Eigen::Matrix<double, N_STATE, N_STATE> P;
    Eigen::Matrix<double, N_STATE, N_STATE> Q;
    Eigen::Matrix<double, N_MEAS,  N_MEAS>  R;
};

}
