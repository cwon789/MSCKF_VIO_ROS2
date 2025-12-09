
#ifndef MSCKF_VIO_IMAGE_PROCESSOR_CORE_H
#define MSCKF_VIO_IMAGE_PROCESSOR_CORE_H

#include <vector>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>

namespace msckf_vio {
namespace core {

/**
 * @brief ImageProcessorCore Pure algorithm implementation for feature detection and tracking
 * ROS-independent core logic for stereo feature processing
 */
class ImageProcessorCore {
public:
  /**
   * @brief ProcessorConfig Configuration parameters for feature detection and tracking
   */
  struct ProcessorConfig {
    int grid_row;
    int grid_col;
    int grid_min_feature_num;
    int grid_max_feature_num;

    int pyramid_levels;
    int patch_size;
    int fast_threshold;
    int max_iteration;
    double track_precision;
    double ransac_threshold;
    double stereo_threshold;
  };

  /**
   * @brief FeatureIDType An alias for unsigned long long int
   */
  typedef unsigned long long int FeatureIDType;

  /**
   * @brief FeatureMetaData Contains necessary information of a feature for easy access
   */
  struct FeatureMetaData {
    FeatureIDType id;
    float response;
    int lifetime;
    cv::Point2f cam0_point;
    cv::Point2f cam1_point;
  };

  /**
   * @brief GridFeatures Organize features based on the grid they belong to
   * Note that the key is encoded by the grid index
   */
  typedef std::map<int, std::vector<FeatureMetaData>> GridFeatures;

  // Constructor
  ImageProcessorCore() : is_first_img(true), next_feature_id(0),
                         before_tracking(0), after_tracking(0),
                         after_matching(0), after_ransac(0) {}

  // Destructor
  virtual ~ImageProcessorCore() = default;

  /**
   * @brief Static comparison functions
   */
  static bool keyPointCompareByResponse(const cv::KeyPoint& pt1, const cv::KeyPoint& pt2) {
    return pt1.response > pt2.response;
  }

  static bool featureCompareByResponse(const FeatureMetaData& f1, const FeatureMetaData& f2) {
    return f1.response > f2.response;
  }

  static bool featureCompareByLifetime(const FeatureMetaData& f1, const FeatureMetaData& f2) {
    return f1.lifetime > f2.lifetime;
  }

  /**
   * @brief Initialize first frame - detect features on first stereo image pair
   */
  void initializeFirstFrame();

  /**
   * @brief Track features on newly received stereo images
   */
  void trackFeatures();

  /**
   * @brief Detect new features to ensure uniform distribution
   */
  void addNewFeatures();

  /**
   * @brief Remove excess features from grids to maintain bounded number
   */
  void pruneGridFeatures();

  /**
   * @brief Create image pyramids for KLT tracking
   */
  void createImagePyramids();

  /**
   * @brief Integrate IMU data between consecutive frames
   * @param cam0_R_p_c Rotation from previous cam0 to current cam0
   * @param cam1_R_p_c Rotation from previous cam1 to current cam1
   */
  void integrateImuData(cv::Matx33f& cam0_R_p_c, cv::Matx33f& cam1_R_p_c);

  /**
   * @brief Predict feature tracking using rotation compensation
   * @param input_pts Features in previous image
   * @param R_p_c Rotation from previous to current camera frame
   * @param intrinsics Camera intrinsics
   * @param compensated_pts Predicted feature locations
   */
  void predictFeatureTracking(
      const std::vector<cv::Point2f>& input_pts,
      const cv::Matx33f& R_p_c,
      const cv::Vec4d& intrinsics,
      std::vector<cv::Point2f>& compensated_pts);

  /**
   * @brief Two-point RANSAC for outlier rejection
   * @param pts1 First set of points
   * @param pts2 Second set of points
   * @param R_p_c Rotation from previous to current frame
   * @param intrinsics Camera intrinsics
   * @param distortion_model Distortion model name
   * @param distortion_coeffs Distortion coefficients
   * @param inlier_error Acceptable inlier error
   * @param success_probability Required probability of success
   * @param inlier_markers Output: 1 for inliers, 0 for outliers
   */
  void twoPointRansac(
      const std::vector<cv::Point2f>& pts1,
      const std::vector<cv::Point2f>& pts2,
      const cv::Matx33f& R_p_c,
      const cv::Vec4d& intrinsics,
      const std::string& distortion_model,
      const cv::Vec4d& distortion_coeffs,
      const double& inlier_error,
      const double& success_probability,
      std::vector<int>& inlier_markers);

  /**
   * @brief Undistort points using camera model
   */
  void undistortPoints(
      const std::vector<cv::Point2f>& pts_in,
      const cv::Vec4d& intrinsics,
      const std::string& distortion_model,
      const cv::Vec4d& distortion_coeffs,
      std::vector<cv::Point2f>& pts_out,
      const cv::Matx33d& rectification_matrix = cv::Matx33d::eye(),
      const cv::Vec4d& new_intrinsics = cv::Vec4d(1, 1, 0, 0));

  /**
   * @brief Distort points using camera model
   */
  std::vector<cv::Point2f> distortPoints(
      const std::vector<cv::Point2f>& pts_in,
      const cv::Vec4d& intrinsics,
      const std::string& distortion_model,
      const cv::Vec4d& distortion_coeffs);

  /**
   * @brief Rescale points for numerical stability
   */
  void rescalePoints(
      std::vector<cv::Point2f>& pts1,
      std::vector<cv::Point2f>& pts2,
      float& scaling_factor);

  /**
   * @brief Stereo matching between cam0 and cam1
   * @param cam0_points Points in primary image
   * @param cam1_points Points in secondary image (output)
   * @param inlier_markers Valid matches (output)
   */
  void stereoMatch(
      const std::vector<cv::Point2f>& cam0_points,
      std::vector<cv::Point2f>& cam1_points,
      std::vector<unsigned char>& inlier_markers);

  /**
   * @brief Remove unmarked elements from vector
   * @param raw_vec Input vector with outliers
   * @param markers 0 for outliers, 1 for inliers
   * @param refined_vec Output vector without outliers
   */
  template <typename T>
  void removeUnmarkedElements(
      const std::vector<T>& raw_vec,
      const std::vector<unsigned char>& markers,
      std::vector<T>& refined_vec) {
    for (size_t i = 0; i < markers.size(); ++i) {
      if (markers[i] == 0) continue;
      refined_vec.push_back(raw_vec[i]);
    }
  }

protected:
  // Indicate if this is the first image message
  bool is_first_img;

  // ID for the next new feature
  FeatureIDType next_feature_id;

  // Feature detector
  ProcessorConfig processor_config;
  cv::Ptr<cv::Feature2D> detector_ptr;

  // Camera calibration parameters
  std::string cam0_distortion_model;
  cv::Vec2i cam0_resolution;
  cv::Vec4d cam0_intrinsics;
  cv::Vec4d cam0_distortion_coeffs;

  std::string cam1_distortion_model;
  cv::Vec2i cam1_resolution;
  cv::Vec4d cam1_intrinsics;
  cv::Vec4d cam1_distortion_coeffs;

  // Take a vector from cam0/cam1 frame to the IMU frame
  cv::Matx33d R_cam0_imu;
  cv::Vec3d t_cam0_imu;
  cv::Matx33d R_cam1_imu;
  cv::Vec3d t_cam1_imu;

  // Previous and current images
  cv::Mat cam0_prev_img;
  cv::Mat cam0_curr_img;
  cv::Mat cam1_curr_img;
  double cam0_prev_time;
  double cam0_curr_time;

  // Pyramids for previous and current image
  std::vector<cv::Mat> prev_cam0_pyramid_;
  std::vector<cv::Mat> curr_cam0_pyramid_;
  std::vector<cv::Mat> curr_cam1_pyramid_;

  // Features in the previous and current image
  std::shared_ptr<GridFeatures> prev_features_ptr;
  std::shared_ptr<GridFeatures> curr_features_ptr;

  // Number of features after each outlier removal step
  int before_tracking;
  int after_tracking;
  int after_matching;
  int after_ransac;

  // IMU data buffer (for integration)
  struct ImuData {
    double time;
    cv::Vec3f angular_velocity;
  };
  std::vector<ImuData> imu_data_buffer;

  // Debugging
  std::map<FeatureIDType, int> feature_lifetime;
};

}  // namespace core
}  // namespace msckf_vio

#endif  // MSCKF_VIO_IMAGE_PROCESSOR_CORE_H
