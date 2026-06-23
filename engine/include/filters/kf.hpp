#pragma once

#include "filters/filter_base.hpp"

namespace orbitforge::filters {

// See math.md §3.1.
class KalmanFilter : public FilterBase<6, 3> {
public:
    Eigen::Matrix<double, 3, 6> H;

    KalmanFilter();

    void predict(double dt) override;
    void update(const Eigen::Matrix<double, 3, 1>& z) override;
};

}
