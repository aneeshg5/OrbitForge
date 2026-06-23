#pragma once

#include "filters/filter_base.hpp"
#include "dynamics/eom.hpp"
#include "dynamics/rigid_body.hpp"
#include "math/quaternion.hpp"

namespace orbitforge::filters {

// See math.md §7.3.
class ExtendedKalmanFilter : public FilterBase<12, 3> {
public:
    Eigen::Matrix<double, 3, 12> H;
    dynamics::PerturbationConfig perturb_cfg;
    dynamics::InertiaTensor      inertia;
    double                       julian_date;
    math::Quat                   q_ref;

    ExtendedKalmanFilter();

    // F block-diagonal between attitude and orbital blocks (math.md §7.3).
    void predict(double dt) override;
    void update(const Eigen::Matrix<double, 3, 1>& z) override;

    // MEKF reset step (math.md §7.3):
    //   q_ref <- (q_ref ⊗ quat_exp(delta_theta)).normalized();  delta_theta <- 0
    void reset_attitude_error();
};

}
