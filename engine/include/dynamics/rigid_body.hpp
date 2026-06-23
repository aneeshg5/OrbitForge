#pragma once

#include <Eigen/Dense>

namespace orbitforge::dynamics {

struct InertiaTensor {
    double ix = 1.0;
    double iy = 1.0;
    double iz = 1.0;
};

using AttitudeState = Eigen::Matrix<double, 7, 1>;

// ẋ = [q̇; ω̇] for torque-free rigid-body rotation (docs/math.md §4.1-4.2):
//   q̇ = ½·q⊗[0,ω]                          (quaternion kinematics)
//   ω̇ = I⁻¹·(-ω×(I·ω))                      (Euler's equation, τ_ext = 0)
AttitudeState attitude_derivative(const AttitudeState& x, const InertiaTensor& inertia);

// ∂ω̇/∂ω (docs/math.md §4.2):
//   ∂ω̇/∂ω = -I⁻¹·([ω×]·I - [(I·ω)×])
Eigen::Matrix3d euler_jacobian(const Eigen::Vector3d& omega, const InertiaTensor& inertia);

}
