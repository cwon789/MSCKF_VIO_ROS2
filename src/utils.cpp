/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <msckf_vio/utils.h>
#include <vector>

namespace msckf_vio {
namespace utils {

Eigen::Isometry3d getTransformEigen(
    rclcpp::Node &node,
    const std::string &field) {
  Eigen::Isometry3d T;
  cv::Mat c = getTransformCV(node, field);

  T.linear()(0, 0)   = c.at<double>(0, 0);
  T.linear()(0, 1)   = c.at<double>(0, 1);
  T.linear()(0, 2)   = c.at<double>(0, 2);
  T.linear()(1, 0)   = c.at<double>(1, 0);
  T.linear()(1, 1)   = c.at<double>(1, 1);
  T.linear()(1, 2)   = c.at<double>(1, 2);
  T.linear()(2, 0)   = c.at<double>(2, 0);
  T.linear()(2, 1)   = c.at<double>(2, 1);
  T.linear()(2, 2)   = c.at<double>(2, 2);
  T.translation()(0) = c.at<double>(0, 3);
  T.translation()(1) = c.at<double>(1, 3);
  T.translation()(2) = c.at<double>(2, 3);
  return T;
}

cv::Mat getTransformCV(
    rclcpp::Node &node,
    const std::string &field) {
  try {
    return getVec16Transform(node, field);
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR(node.get_logger(),
      "Cannot read transform %s: %s", field.c_str(), e.what());
    throw;
  }
}

cv::Mat getVec16Transform(
    rclcpp::Node &node,
    const std::string &field) {
  std::vector<double> v;
  if (!node.has_parameter(field)) {
    node.declare_parameter<std::vector<double>>(field, {});
  }
  node.get_parameter(field, v);
  if (v.size() != 16) {
    throw std::runtime_error("Parameter " + field + " must contain 16 values");
  }
  cv::Mat T = cv::Mat(v).clone().reshape(1, 4); // one channel 4 rows
  return T;
}

} // namespace utils
} // namespace msckf_vio
