/*
 * ROS Message Conversion Utilities
 * Conversions between ROS messages and Eigen types
 */

#ifndef MSCKF_VIO_ROS_CONVERSIONS_H
#define MSCKF_VIO_ROS_CONVERSIONS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace msckf_vio {
namespace ros_utils {

inline double toSec(const builtin_interfaces::msg::Time &t) {
  return rclcpp::Time(t).seconds();
}

inline void vectorMsgToEigen(const geometry_msgs::msg::Vector3 &msg, Eigen::Vector3d &vec) {
  vec = Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline void pointMsgToEigen(const geometry_msgs::msg::Point &msg, Eigen::Vector3d &vec) {
  vec = Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion &msg, Eigen::Quaterniond &quat) {
  quat = Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

inline void poseEigenToMsg(const Eigen::Isometry3d &pose, geometry_msgs::msg::Pose &msg) {
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  Eigen::Quaterniond q(pose.rotation());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
}

inline void vectorEigenToMsg(const Eigen::Vector3d &vec, geometry_msgs::msg::Vector3 &msg) {
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
}

inline geometry_msgs::msg::Transform toTransformMsg(const Eigen::Isometry3d &pose) {
  geometry_msgs::msg::Transform tf_msg;
  Eigen::Quaterniond q(pose.rotation());
  tf_msg.translation.x = pose.translation().x();
  tf_msg.translation.y = pose.translation().y();
  tf_msg.translation.z = pose.translation().z();
  tf_msg.rotation.x = q.x();
  tf_msg.rotation.y = q.y();
  tf_msg.rotation.z = q.z();
  tf_msg.rotation.w = q.w();
  return tf_msg;
}

}  // namespace ros_utils
}  // namespace msckf_vio

#endif  // MSCKF_VIO_ROS_CONVERSIONS_H
