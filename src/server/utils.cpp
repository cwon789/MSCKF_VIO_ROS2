
#include <msckf_vio/utils/utils.h>
#include <msckf_vio/utils/conversion.h>
#include <vector>

namespace msckf_vio {
namespace utils {

// ROS wrapper - uses core conversion utilities
Eigen::Isometry3d getTransformEigen(
    rclcpp::Node &node,
    const std::string &field) {
  cv::Mat c = getTransformCV(node, field);
  return cvMatToIsometry3d(c);  // Use ROS-independent core utility
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

  // Use ROS-independent core utility
  return vectorToCvMat(v);
}

} // namespace utils
} // namespace msckf_vio
