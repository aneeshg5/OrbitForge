#include "dynamics/rigid_body.hpp"
#include "math/quaternion.hpp"

namespace orbitforge::dynamics {

AttitudeState attitude_derivative(const AttitudeState& x, const InertiaTensor& inertia) {
    // Quat ctor order is (w,x,y,z); x's first 4 entries are coeffs()
    // order (x,y,z,w) — see math/quaternion.hpp and AttitudeState's doc.
    const math::Quat q(x[3], x[0], x[1], x[2]);
    const Eigen::Vector3d omega = x.tail<3>();
    const Eigen::Vector3d i_diag(inertia.ix, inertia.iy, inertia.iz);
    const Eigen::Vector3d i_omega = i_diag.cwiseProduct(omega);

    AttitudeState dx;
    dx.head<4>() = math::quat_kinematics_coeffs_dot(q, omega);
    // ω̇ = I⁻¹·(-ω×(I·ω)) = -(ω×(I·ω)) ./ I  (I diagonal, so I⁻¹ is elementwise)
    dx.tail<3>() = (-omega.cross(i_omega)).cwiseQuotient(i_diag);
    return dx;
}

Eigen::Matrix3d euler_jacobian(const Eigen::Vector3d& omega, const InertiaTensor& inertia) {
    const Eigen::Vector3d i_diag(inertia.ix, inertia.iy, inertia.iz);
    const Eigen::Matrix3d i_mat = i_diag.asDiagonal();
    const Eigen::Vector3d i_omega = i_diag.cwiseProduct(omega);

    // ∂(ω×(Iω))/∂ω = [ω×]·I - [(Iω)×]  (docs/math.md §4.2)
    const Eigen::Matrix3d dg_domega = math::skew(omega) * i_mat - math::skew(i_omega);

    const Eigen::Vector3d i_inv_diag = i_diag.cwiseInverse();
    return -Eigen::Matrix3d(i_inv_diag.asDiagonal()) * dg_domega;
}

} // namespace orbitforge::dynamics
