#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace orbitforge::math {

// docs/math.md §4.
using Quat = Eigen::Quaterniond;

Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// docs/math.md §4.3.
Quat quat_exp(const Eigen::Vector3d& delta_theta);

Eigen::Vector3d quat_log(const Quat& q);

// docs/math.md §4.1.
Eigen::Vector4d quat_kinematics_coeffs_dot(const Quat& q, const Eigen::Vector3d& omega_body);

}
