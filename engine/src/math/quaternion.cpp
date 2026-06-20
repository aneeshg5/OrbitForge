#include "math/quaternion.hpp"

namespace orbitforge::math {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<      0.0, -v.z(),  v.y(),
            v.z(),    0.0, -v.x(),
           -v.y(),  v.x(),    0.0;
    return m;
}

Quat quat_exp(const Eigen::Vector3d& delta_theta) {
    const double angle = delta_theta.norm();
    if (angle < 1e-12) {
        // First-order Taylor expansion of [cos(a/2), sin(a/2)*axis] as
        // a -> 0, avoiding the 0/0 in axis = delta_theta/angle.
        return Quat(1.0, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z()).normalized();
    }
    const Eigen::Vector3d axis = delta_theta / angle;
    return Quat(Eigen::AngleAxisd(angle, axis));
}

Eigen::Vector3d quat_log(const Quat& q) {
    const Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

Eigen::Vector4d quat_kinematics_coeffs_dot(const Quat& q, const Eigen::Vector3d& omega_body) {
    // q̇ = ½·q⊗[0,ω]  (docs/math.md §4.1). Quat ctor order is (w,x,y,z);
    // the pure-vector quaternion [0,ω] has zero scalar part.
    const Quat omega_quat(0.0, omega_body.x(), omega_body.y(), omega_body.z());
    const Quat qdot = q * omega_quat;  // Hamilton product; result is not unit-norm, as expected for a derivative
    return 0.5 * qdot.coeffs();
}

} // namespace orbitforge::math
