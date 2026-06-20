#pragma once

#include <Eigen/Dense>

namespace orbitforge::dynamics {

// Principal-axis inertia tensor (diagonal only — off-diagonal products of
// inertia are not modeled, §18). kg·m².
struct InertiaTensor {
    double ix = 1.0;
    double iy = 1.0;
    double iz = 1.0;
};

// Combined attitude state for RK4 integration of the "true" trajectory:
// [qx, qy, qz, qw, wx, wy, wz] — quaternion coefficients in
// Eigen::Quaterniond::coeffs() order (NOT constructor order, see
// math/quaternion.hpp) followed by body-frame angular velocity (rad/s).
// A plain 7-vector (rather than a Quat + Vector3d pair) so it can be
// stepped through the existing rk4_step<State> template, which requires
// State to support operator+ and scalar operator* — Eigen::Quaterniond
// itself does not.
using AttitudeState = Eigen::Matrix<double, 7, 1>;

// ẋ = [q̇; ω̇] for torque-free rigid-body rotation (docs/math.md §4.1-4.2):
//   q̇ = ½·q⊗[0,ω]                          (quaternion kinematics)
//   ω̇ = I⁻¹·(-ω×(I·ω))                      (Euler's equation, τ_ext = 0 — §18)
AttitudeState attitude_derivative(const AttitudeState& x, const InertiaTensor& inertia);

// ∂ω̇/∂ω (docs/math.md §4.2), the only nonzero block of the attitude
// Jacobian's ω-row (ω̇ doesn't depend on attitude at all — torque-free
// rotation is attitude-independent):
//   ∂ω̇/∂ω = -I⁻¹·([ω×]·I - [(I·ω)×])
Eigen::Matrix3d euler_jacobian(const Eigen::Vector3d& omega, const InertiaTensor& inertia);

} // namespace orbitforge::dynamics
