#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace orbitforge::math {

// Attitude quaternion convention (Phase 5, see docs/math.md §4 and
// CLAUDE.md §6): q represents the rotation from BODY to ECI — for a
// body-frame vector v_b, its ECI components are v_i = q*v_b (Eigen's
// native Quaterniond-vector product). The ECI->body rotation matrix used
// by the gyro/magnetometer measurement models is therefore
// q.conjugate().toRotationMatrix(), equivalently q.toRotationMatrix().transpose().
//
// Composition order: A(q1*q2) = A(q2)*A(q1) where A(q) = R(q)^T is the
// ECI->body DCM (composition reverses relative to the quaternion product —
// this is what makes the MEKF reset step below q_ref ⊗ δq, not δq ⊗ q_ref).
using Quat = Eigen::Quaterniond;

// Skew-symmetric (cross-product) matrix: skew(v)*x == v.cross(x).
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// Exponential map: small rotation vector delta_theta (rad, body frame) ->
// the unit quaternion it generates. Exact (uses the true cos/sin axis-angle
// form, not just a first-order approximation), with a safe first-order
// fallback near delta_theta == 0 to avoid a 0/0 in the axis normalization.
//
// Feeds the MEKF reset step (docs/math.md §4.3): after computing the
// posterior attitude-error estimate delta_theta_hat, q_ref is corrected via
//   q_ref <- (q_ref * quat_exp(delta_theta_hat)).normalized()
// (right-multiplication — the error is defined in the reference/body frame,
// q_true = q_ref ⊗ δq, the standard Lefferts-Markley-Shuster MEKF
// convention), then delta_theta_hat is reset to zero.
Quat quat_exp(const Eigen::Vector3d& delta_theta);

// Logarithmic map, the inverse of quat_exp: recovers the rotation vector
// (rad) from a unit quaternion.
Eigen::Vector3d quat_log(const Quat& q);

// d(q.coeffs())/dt for the quaternion kinematics q̇ = ½·q⊗[0,ω] (docs/math.md
// §4.1), where omega_body is angular velocity expressed in the body frame.
// Returned as a plain Vector4d in the SAME [x,y,z,w] coefficient order as
// Eigen::Quaterniond::coeffs() (note this differs from the (w,x,y,z)
// *constructor* argument order — an Eigen gotcha worth flagging explicitly).
// This lets the "true" attitude trajectory integrate q through the existing
// rk4_step<State> (which requires State to support operator+ and scalar
// operator*, which Eigen::Quaterniond itself does not) by treating the
// coefficients as a plain 4-vector; see dynamics/rigid_body.hpp.
Eigen::Vector4d quat_kinematics_coeffs_dot(const Quat& q, const Eigen::Vector3d& omega_body);

} // namespace orbitforge::math
