
#ifndef MSCKF_VIO_CONVERSION_H
#define MSCKF_VIO_CONVERSION_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace msckf_vio {
namespace utils {

/**
 * @brief Convert 16-element vector to 4x4 transformation matrix
 * ROS-independent utility
 */
inline Eigen::Isometry3d vectorToIsometry3d(const std::vector<double>& vec) {
  if (vec.size() != 16) {
    throw std::runtime_error("Vector must contain exactly 16 elements for 4x4 transform");
  }

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // Fill rotation part (3x3)
  T.linear()(0, 0) = vec[0];  T.linear()(0, 1) = vec[1];  T.linear()(0, 2) = vec[2];
  T.linear()(1, 0) = vec[4];  T.linear()(1, 1) = vec[5];  T.linear()(1, 2) = vec[6];
  T.linear()(2, 0) = vec[8];  T.linear()(2, 1) = vec[9];  T.linear()(2, 2) = vec[10];

  // Fill translation part
  T.translation()(0) = vec[3];
  T.translation()(1) = vec[7];
  T.translation()(2) = vec[11];

  return T;
}

/**
 * @brief Convert OpenCV Mat (4x4) to Eigen::Isometry3d
 * ROS-independent utility
 */
inline Eigen::Isometry3d cvMatToIsometry3d(const cv::Mat& mat) {
  if (mat.rows != 4 || mat.cols != 4) {
    throw std::runtime_error("Matrix must be 4x4 for transformation");
  }

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // Fill rotation and translation
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      T.linear()(i, j) = mat.at<double>(i, j);
    }
    T.translation()(i) = mat.at<double>(i, 3);
  }

  return T;
}

/**
 * @brief Convert vector to OpenCV Mat (4x4)
 * ROS-independent utility
 */
inline cv::Mat vectorToCvMat(const std::vector<double>& vec) {
  if (vec.size() != 16) {
    throw std::runtime_error("Vector must contain exactly 16 elements");
  }

  cv::Mat T = cv::Mat(vec).clone().reshape(1, 4); // one channel, 4 rows
  return T;
}

} // namespace utils
} // namespace msckf_vio

#endif // MSCKF_VIO_CONVERSION_H
