#pragma once

#include "filters/filter_base.hpp"
#include "dynamics/eom.hpp"
#include "dynamics/rigid_body.hpp"
#include "math/quaternion.hpp"

namespace orbitforge::filters {

// See math.md §7.3.
class UnscentedKalmanFilter : public FilterBase<12, 3> {
public:
    Eigen::Matrix<double, 12, 12> S;
    Eigen::Matrix<double, 3, 12>  H;
    dynamics::PerturbationConfig  perturb_cfg;
    dynamics::InertiaTensor       inertia;
    double                        julian_date;
    math::Quat                    q_ref;

    UnscentedKalmanFilter();

    void predict(double dt) override;
    void update(const Eigen::Matrix<double, 3, 1>& z) override;

    // MEKF reset step (math.md §7.3).
    void reset_attitude_error();
};

}
