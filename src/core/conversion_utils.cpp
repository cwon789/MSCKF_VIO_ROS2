/*
 * Core Conversion Utilities
 * ROS-independent data conversion functions
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace msckf_vio {
namespace core {

// Pure Eigen conversions - no ROS dependencies

/**
 * @brief Convert Isometry3d to 4x4 matrix
 */
inline Eigen::Matrix4d isometry3dToMatrix4d(const Eigen::Isometry3d& transform) {
  return transform.matrix();
}

/**
 * @brief Convert quaternion to rotation matrix
 */
inline Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& q) {
  Eigen::Quaterniond quat(q(3), q(0), q(1), q(2)); // w, x, y, z
  return quat.toRotationMatrix();
}

/**
 * @brief Convert rotation matrix to quaternion
 */
inline Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

} // namespace core
} // namespace msckf_vio
