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
    // q̇ = ½·q⊗[0,ω]  (docs/math.md §4.1).
    const Quat omega_quat(0.0, omega_body.x(), omega_body.y(), omega_body.z());
    const Quat qdot = q * omega_quat;
    return 0.5 * qdot.coeffs();
}

}
