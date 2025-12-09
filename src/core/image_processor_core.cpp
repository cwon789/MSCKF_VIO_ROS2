/**
 * @file image_processor_core.cpp
 * @brief Core image processing algorithms without ROS dependencies
 */

#include <iostream>
#include <algorithm>
#include <set>
#include <random>
#include <Eigen/Dense>
#include <msckf_vio/core/image/image_processor_core.h>

using namespace std;
using namespace cv;

namespace msckf_vio {
namespace core {

void ImageProcessorCore::createImagePyramids() {
  buildOpticalFlowPyramid(
      cam0_curr_img, curr_cam0_pyramid_,
      Size(processor_config.patch_size, processor_config.patch_size),
      processor_config.pyramid_levels, true, BORDER_REFLECT_101,
      BORDER_CONSTANT, false);

  buildOpticalFlowPyramid(
      cam1_curr_img, curr_cam1_pyramid_,
      Size(processor_config.patch_size, processor_config.patch_size),
      processor_config.pyramid_levels, true, BORDER_REFLECT_101,
      BORDER_CONSTANT, false);
}

void ImageProcessorCore::initializeFirstFrame() {
  // Size of each grid
  static int grid_height = cam0_curr_img.rows / processor_config.grid_row;
  static int grid_width = cam0_curr_img.cols / processor_config.grid_col;

  // Detect new features on the first image
  vector<KeyPoint> new_features(0);
  detector_ptr->detect(cam0_curr_img, new_features);

  // Find the stereo matched points for the newly detected features
  vector<Point2f> cam0_points(new_features.size());
  for (size_t i = 0; i < new_features.size(); ++i)
    cam0_points[i] = new_features[i].pt;

  vector<Point2f> cam1_points(0);
  vector<unsigned char> inlier_markers(0);
  stereoMatch(cam0_points, cam1_points, inlier_markers);

  vector<Point2f> cam0_inliers(0);
  vector<Point2f> cam1_inliers(0);
  vector<float> response_inliers(0);
  for (size_t i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    cam0_inliers.push_back(cam0_points[i]);
    cam1_inliers.push_back(cam1_points[i]);
    response_inliers.push_back(new_features[i].response);
  }

  // Group the features into grids
  GridFeatures grid_new_features;
  for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code)
    grid_new_features[code] = vector<FeatureMetaData>(0);

  for (size_t i = 0; i < cam0_inliers.size(); ++i) {
    const Point2f& cam0_point = cam0_inliers[i];
    const Point2f& cam1_point = cam1_inliers[i];
    const float& response = response_inliers[i];

    int row = static_cast<int>(cam0_point.y / grid_height);
    int col = static_cast<int>(cam0_point.x / grid_width);
    int code = row * processor_config.grid_col + col;

    FeatureMetaData new_feature;
    new_feature.response = response;
    new_feature.cam0_point = cam0_point;
    new_feature.cam1_point = cam1_point;
    grid_new_features[code].push_back(new_feature);
  }

  // Sort the new features in each grid based on its response
  for (auto& item : grid_new_features)
    std::sort(item.second.begin(), item.second.end(),
              &ImageProcessorCore::featureCompareByResponse);

  // Collect new features within each grid with high response
  for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code) {
    vector<FeatureMetaData>& features_this_grid = (*curr_features_ptr)[code];
    vector<FeatureMetaData>& new_features_this_grid = grid_new_features[code];

    for (int k = 0; k < processor_config.grid_min_feature_num &&
                    k < static_cast<int>(new_features_this_grid.size()); ++k) {
      features_this_grid.push_back(new_features_this_grid[k]);
      features_this_grid.back().id = next_feature_id++;
      features_this_grid.back().lifetime = 1;
    }
  }
}

void ImageProcessorCore::predictFeatureTracking(
    const vector<Point2f>& input_pts,
    const Matx33f& R_p_c,
    const Vec4d& intrinsics,
    vector<Point2f>& compensated_pts) {

  // Return directly if there are no input features
  if (input_pts.size() == 0) {
    compensated_pts.clear();
    return;
  }
  compensated_pts.resize(input_pts.size());

  // Intrinsic matrix
  Matx33f K(
      intrinsics[0], 0.0, intrinsics[2],
      0.0, intrinsics[1], intrinsics[3],
      0.0, 0.0, 1.0);
  Matx33f H = K * R_p_c * K.inv();

  for (size_t i = 0; i < input_pts.size(); ++i) {
    Vec3f p1(input_pts[i].x, input_pts[i].y, 1.0f);
    Vec3f p2 = H * p1;
    compensated_pts[i].x = p2[0] / p2[2];
    compensated_pts[i].y = p2[1] / p2[2];
  }
}

void ImageProcessorCore::trackFeatures() {
  // Size of each grid
  static int grid_height = cam0_curr_img.rows / processor_config.grid_row;
  static int grid_width = cam0_curr_img.cols / processor_config.grid_col;

  // Compute a rough relative rotation which takes a vector
  // from the previous frame to the current frame
  Matx33f cam0_R_p_c;
  Matx33f cam1_R_p_c;
  integrateImuData(cam0_R_p_c, cam1_R_p_c);

  // Organize the features in the previous image
  vector<FeatureIDType> prev_ids(0);
  vector<int> prev_lifetime(0);
  vector<Point2f> prev_cam0_points(0);
  vector<Point2f> prev_cam1_points(0);

  for (const auto& item : *prev_features_ptr) {
    for (const auto& prev_feature : item.second) {
      prev_ids.push_back(prev_feature.id);
      prev_lifetime.push_back(prev_feature.lifetime);
      prev_cam0_points.push_back(prev_feature.cam0_point);
      prev_cam1_points.push_back(prev_feature.cam1_point);
    }
  }

  // Number of features before tracking
  before_tracking = prev_cam0_points.size();

  // Abort tracking if there are no features in the previous frame
  if (prev_ids.size() == 0) return;

  // Track features using LK optical flow method
  vector<Point2f> curr_cam0_points(0);
  vector<unsigned char> track_inliers(0);

  predictFeatureTracking(prev_cam0_points, cam0_R_p_c, cam0_intrinsics, curr_cam0_points);

  calcOpticalFlowPyrLK(
      prev_cam0_pyramid_, curr_cam0_pyramid_,
      prev_cam0_points, curr_cam0_points,
      track_inliers, noArray(),
      Size(processor_config.patch_size, processor_config.patch_size),
      processor_config.pyramid_levels,
      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
                   processor_config.max_iteration,
                   processor_config.track_precision),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  // Mark those tracked points out of the image region as untracked
  for (size_t i = 0; i < curr_cam0_points.size(); ++i) {
    if (track_inliers[i] == 0) continue;
    if (curr_cam0_points[i].y < 0 ||
        curr_cam0_points[i].y > cam0_curr_img.rows - 1 ||
        curr_cam0_points[i].x < 0 ||
        curr_cam0_points[i].x > cam0_curr_img.cols - 1)
      track_inliers[i] = 0;
  }

  // Collect the tracked points
  vector<FeatureIDType> prev_tracked_ids(0);
  vector<int> prev_tracked_lifetime(0);
  vector<Point2f> prev_tracked_cam0_points(0);
  vector<Point2f> prev_tracked_cam1_points(0);
  vector<Point2f> curr_tracked_cam0_points(0);

  removeUnmarkedElements(prev_ids, track_inliers, prev_tracked_ids);
  removeUnmarkedElements(prev_lifetime, track_inliers, prev_tracked_lifetime);
  removeUnmarkedElements(prev_cam0_points, track_inliers, prev_tracked_cam0_points);
  removeUnmarkedElements(prev_cam1_points, track_inliers, prev_tracked_cam1_points);
  removeUnmarkedElements(curr_cam0_points, track_inliers, curr_tracked_cam0_points);

  // Number of features left after tracking
  after_tracking = curr_tracked_cam0_points.size();

  // Step 1: stereo matching
  vector<Point2f> curr_cam1_points(0);
  vector<unsigned char> match_inliers(0);
  stereoMatch(curr_tracked_cam0_points, curr_cam1_points, match_inliers);

  vector<FeatureIDType> prev_matched_ids(0);
  vector<int> prev_matched_lifetime(0);
  vector<Point2f> prev_matched_cam0_points(0);
  vector<Point2f> prev_matched_cam1_points(0);
  vector<Point2f> curr_matched_cam0_points(0);
  vector<Point2f> curr_matched_cam1_points(0);

  removeUnmarkedElements(prev_tracked_ids, match_inliers, prev_matched_ids);
  removeUnmarkedElements(prev_tracked_lifetime, match_inliers, prev_matched_lifetime);
  removeUnmarkedElements(prev_tracked_cam0_points, match_inliers, prev_matched_cam0_points);
  removeUnmarkedElements(prev_tracked_cam1_points, match_inliers, prev_matched_cam1_points);
  removeUnmarkedElements(curr_tracked_cam0_points, match_inliers, curr_matched_cam0_points);
  removeUnmarkedElements(curr_cam1_points, match_inliers, curr_matched_cam1_points);

  // Number of features left after stereo matching
  after_matching = curr_matched_cam0_points.size();

  // Step 2 and 3: RANSAC on temporal image pairs of cam0 and cam1
  vector<int> cam0_ransac_inliers(0);
  twoPointRansac(prev_matched_cam0_points, curr_matched_cam0_points,
                 cam0_R_p_c, cam0_intrinsics, cam0_distortion_model,
                 cam0_distortion_coeffs, processor_config.ransac_threshold,
                 0.99, cam0_ransac_inliers);

  vector<int> cam1_ransac_inliers(0);
  twoPointRansac(prev_matched_cam1_points, curr_matched_cam1_points,
                 cam1_R_p_c, cam1_intrinsics, cam1_distortion_model,
                 cam1_distortion_coeffs, processor_config.ransac_threshold,
                 0.99, cam1_ransac_inliers);

  // Number of features after ransac
  after_ransac = 0;

  for (size_t i = 0; i < cam0_ransac_inliers.size(); ++i) {
    if (cam0_ransac_inliers[i] == 0 || cam1_ransac_inliers[i] == 0) continue;
    int row = static_cast<int>(curr_matched_cam0_points[i].y / grid_height);
    int col = static_cast<int>(curr_matched_cam0_points[i].x / grid_width);
    int code = row * processor_config.grid_col + col;
    (*curr_features_ptr)[code].push_back(FeatureMetaData());

    FeatureMetaData& grid_new_feature = (*curr_features_ptr)[code].back();
    grid_new_feature.id = prev_matched_ids[i];
    grid_new_feature.lifetime = ++prev_matched_lifetime[i];
    grid_new_feature.cam0_point = curr_matched_cam0_points[i];
    grid_new_feature.cam1_point = curr_matched_cam1_points[i];

    ++after_ransac;
  }
}

void ImageProcessorCore::stereoMatch(
    const vector<Point2f>& cam0_points,
    vector<Point2f>& cam1_points,
    vector<unsigned char>& inlier_markers) {

  if (cam0_points.size() == 0) return;

  if (cam1_points.size() == 0) {
    // Initialize cam1_points by projecting cam0_points to cam1 using rotation from stereo extrinsics
    const Matx33d R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu;
    vector<Point2f> cam0_points_undistorted;
    undistortPoints(cam0_points, cam0_intrinsics, cam0_distortion_model,
                    cam0_distortion_coeffs, cam0_points_undistorted,
                    R_cam0_cam1);
    cam1_points = distortPoints(cam0_points_undistorted, cam1_intrinsics,
                                cam1_distortion_model, cam1_distortion_coeffs);
  }

  // Track features using LK optical flow method
  calcOpticalFlowPyrLK(curr_cam0_pyramid_, curr_cam1_pyramid_,
                       cam0_points, cam1_points,
                       inlier_markers, noArray(),
                       Size(processor_config.patch_size, processor_config.patch_size),
                       processor_config.pyramid_levels,
                       TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
                                    processor_config.max_iteration,
                                    processor_config.track_precision),
                       cv::OPTFLOW_USE_INITIAL_FLOW);

  // Mark those tracked points out of the image region as untracked
  for (size_t i = 0; i < cam1_points.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    if (cam1_points[i].y < 0 ||
        cam1_points[i].y > cam1_curr_img.rows - 1 ||
        cam1_points[i].x < 0 ||
        cam1_points[i].x > cam1_curr_img.cols - 1)
      inlier_markers[i] = 0;
  }

  // Compute the relative rotation between the cam0 frame and cam1 frame
  const Matx33d R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu;
  const Vec3d t_cam0_cam1 = R_cam1_imu.t() * (t_cam0_imu - t_cam1_imu);
  // Compute the essential matrix
  const Matx33d t_cam0_cam1_hat(
      0.0, -t_cam0_cam1[2], t_cam0_cam1[1],
      t_cam0_cam1[2], 0.0, -t_cam0_cam1[0],
      -t_cam0_cam1[1], t_cam0_cam1[0], 0.0);
  const Matx33d E = t_cam0_cam1_hat * R_cam0_cam1;

  // Further remove outliers based on the known essential matrix
  vector<Point2f> cam0_points_undistorted(0);
  vector<Point2f> cam1_points_undistorted(0);
  undistortPoints(cam0_points, cam0_intrinsics, cam0_distortion_model,
                  cam0_distortion_coeffs, cam0_points_undistorted);
  undistortPoints(cam1_points, cam1_intrinsics, cam1_distortion_model,
                  cam1_distortion_coeffs, cam1_points_undistorted);

  double norm_pixel_unit = 4.0 / (cam0_intrinsics[0] + cam0_intrinsics[1] +
                                   cam1_intrinsics[0] + cam1_intrinsics[1]);

  for (size_t i = 0; i < cam0_points_undistorted.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    Vec3d pt0(cam0_points_undistorted[i].x,
              cam0_points_undistorted[i].y, 1.0);
    Vec3d pt1(cam1_points_undistorted[i].x,
              cam1_points_undistorted[i].y, 1.0);
    Vec3d epipolar_line = E * pt0;
    double error = fabs((pt1.t() * epipolar_line)[0]) / sqrt(
        epipolar_line[0] * epipolar_line[0] +
        epipolar_line[1] * epipolar_line[1]);
    if (error > processor_config.stereo_threshold * norm_pixel_unit)
      inlier_markers[i] = 0;
  }
}

void ImageProcessorCore::addNewFeatures() {
  const Mat& curr_img = cam0_curr_img;

  // Size of each grid
  static int grid_height = cam0_curr_img.rows / processor_config.grid_row;
  static int grid_width = cam0_curr_img.cols / processor_config.grid_col;

  // Create a mask to avoid redetecting existing features
  Mat mask(curr_img.rows, curr_img.cols, CV_8U, Scalar(1));

  for (const auto& features : *curr_features_ptr) {
    for (const auto& feature : features.second) {
      const int y = static_cast<int>(feature.cam0_point.y);
      const int x = static_cast<int>(feature.cam0_point.x);

      int up_lim = y - 2, bottom_lim = y + 3,
          left_lim = x - 2, right_lim = x + 3;
      if (up_lim < 0) up_lim = 0;
      if (bottom_lim > curr_img.rows) bottom_lim = curr_img.rows;
      if (left_lim < 0) left_lim = 0;
      if (right_lim > curr_img.cols) right_lim = curr_img.cols;

      Range row_range(up_lim, bottom_lim);
      Range col_range(left_lim, right_lim);
      mask(row_range, col_range) = 0;
    }
  }

  // Detect new features
  vector<KeyPoint> new_features(0);
  detector_ptr->detect(curr_img, new_features, mask);

  // Collect the new detected features based on the grid
  vector<vector<KeyPoint>> new_feature_sieve(
      processor_config.grid_row * processor_config.grid_col);
  for (const auto& feature : new_features) {
    int row = static_cast<int>(feature.pt.y / grid_height);
    int col = static_cast<int>(feature.pt.x / grid_width);
    new_feature_sieve[row * processor_config.grid_col + col].push_back(feature);
  }

  new_features.clear();
  for (auto& item : new_feature_sieve) {
    if (item.size() > static_cast<size_t>(processor_config.grid_max_feature_num)) {
      std::sort(item.begin(), item.end(),
                &ImageProcessorCore::keyPointCompareByResponse);
      item.erase(item.begin() + processor_config.grid_max_feature_num, item.end());
    }
    new_features.insert(new_features.end(), item.begin(), item.end());
  }

  // Find the stereo matched points for the newly detected features
  vector<Point2f> cam0_points(new_features.size());
  for (size_t i = 0; i < new_features.size(); ++i)
    cam0_points[i] = new_features[i].pt;

  vector<Point2f> cam1_points(0);
  vector<unsigned char> inlier_markers(0);
  stereoMatch(cam0_points, cam1_points, inlier_markers);

  vector<Point2f> cam0_inliers(0);
  vector<Point2f> cam1_inliers(0);
  vector<float> response_inliers(0);
  for (size_t i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    cam0_inliers.push_back(cam0_points[i]);
    cam1_inliers.push_back(cam1_points[i]);
    response_inliers.push_back(new_features[i].response);
  }

  // Group the features into grids
  GridFeatures grid_new_features;
  for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code)
    grid_new_features[code] = vector<FeatureMetaData>(0);

  for (size_t i = 0; i < cam0_inliers.size(); ++i) {
    const Point2f& cam0_point = cam0_inliers[i];
    const Point2f& cam1_point = cam1_inliers[i];
    const float& response = response_inliers[i];

    int row = static_cast<int>(cam0_point.y / grid_height);
    int col = static_cast<int>(cam0_point.x / grid_width);
    int code = row * processor_config.grid_col + col;

    FeatureMetaData new_feature;
    new_feature.response = response;
    new_feature.cam0_point = cam0_point;
    new_feature.cam1_point = cam1_point;
    grid_new_features[code].push_back(new_feature);
  }

  // Sort the new features in each grid based on its response
  for (auto& item : grid_new_features)
    std::sort(item.second.begin(), item.second.end(),
              &ImageProcessorCore::featureCompareByResponse);

  // Collect new features within each grid with high response
  for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code) {
    vector<FeatureMetaData>& features_this_grid = (*curr_features_ptr)[code];
    vector<FeatureMetaData>& new_features_this_grid = grid_new_features[code];

    if (features_this_grid.size() >= static_cast<size_t>(processor_config.grid_min_feature_num))
      continue;

    int vacancy_num = processor_config.grid_min_feature_num - features_this_grid.size();
    for (int k = 0; k < vacancy_num && k < static_cast<int>(new_features_this_grid.size()); ++k) {
      features_this_grid.push_back(new_features_this_grid[k]);
      features_this_grid.back().id = next_feature_id++;
      features_this_grid.back().lifetime = 1;
    }
  }
}

void ImageProcessorCore::pruneGridFeatures() {
  for (auto& item : *curr_features_ptr) {
    auto& grid_features = item.second;
    if (grid_features.size() <= static_cast<size_t>(processor_config.grid_max_feature_num))
      continue;
    std::sort(grid_features.begin(), grid_features.end(),
              &ImageProcessorCore::featureCompareByLifetime);
    grid_features.erase(grid_features.begin() + processor_config.grid_max_feature_num,
                       grid_features.end());
  }
}

void ImageProcessorCore::undistortPoints(
    const vector<Point2f>& pts_in,
    const Vec4d& intrinsics,
    const string& distortion_model,
    const Vec4d& distortion_coeffs,
    vector<Point2f>& pts_out,
    const Matx33d& rectification_matrix,
    const Vec4d& new_intrinsics) {

  if (pts_in.size() == 0) return;

  const Matx33d K(
      intrinsics[0], 0.0, intrinsics[2],
      0.0, intrinsics[1], intrinsics[3],
      0.0, 0.0, 1.0);

  const Matx33d K_new(
      new_intrinsics[0], 0.0, new_intrinsics[2],
      0.0, new_intrinsics[1], new_intrinsics[3],
      0.0, 0.0, 1.0);

  if (distortion_model == "radtan") {
    cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
                        rectification_matrix, K_new);
  } else if (distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
                                 rectification_matrix, K_new);
  } else {
    // Default to radtan
    cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
                        rectification_matrix, K_new);
  }
}

vector<Point2f> ImageProcessorCore::distortPoints(
    const vector<Point2f>& pts_in,
    const Vec4d& intrinsics,
    const string& distortion_model,
    const Vec4d& distortion_coeffs) {

  const Matx33d K(intrinsics[0], 0.0, intrinsics[2],
                  0.0, intrinsics[1], intrinsics[3],
                  0.0, 0.0, 1.0);

  vector<Point2f> pts_out;
  if (distortion_model == "radtan") {
    vector<Point3f> homogenous_pts;
    cv::convertPointsToHomogeneous(pts_in, homogenous_pts);
    cv::projectPoints(homogenous_pts, Vec3d::zeros(), Vec3d::zeros(), K,
                      distortion_coeffs, pts_out);
  } else if (distortion_model == "equidistant") {
    cv::fisheye::distortPoints(pts_in, pts_out, K, distortion_coeffs);
  } else {
    // Default to radtan
    vector<Point3f> homogenous_pts;
    cv::convertPointsToHomogeneous(pts_in, homogenous_pts);
    cv::projectPoints(homogenous_pts, Vec3d::zeros(), Vec3d::zeros(), K,
                      distortion_coeffs, pts_out);
  }

  return pts_out;
}

void ImageProcessorCore::integrateImuData(Matx33f& cam0_R_p_c, Matx33f& cam1_R_p_c) {
  // Find the start and the end limit within the imu data buffer
  auto begin_iter = imu_data_buffer.begin();
  while (begin_iter != imu_data_buffer.end()) {
    if ((begin_iter->time - cam0_prev_time) < -0.01)
      ++begin_iter;
    else
      break;
  }

  auto end_iter = begin_iter;
  while (end_iter != imu_data_buffer.end()) {
    if ((end_iter->time - cam0_curr_time) < 0.005)
      ++end_iter;
    else
      break;
  }

  // Compute the mean angular velocity in the IMU frame
  Vec3f mean_ang_vel(0.0, 0.0, 0.0);
  for (auto iter = begin_iter; iter < end_iter; ++iter)
    mean_ang_vel += iter->angular_velocity;

  if (end_iter - begin_iter > 0)
    mean_ang_vel *= 1.0f / (end_iter - begin_iter);

  // Transform the mean angular velocity from the IMU frame to the cam0 and cam1 frames
  Vec3f cam0_mean_ang_vel = R_cam0_imu.t() * mean_ang_vel;
  Vec3f cam1_mean_ang_vel = R_cam1_imu.t() * mean_ang_vel;

  // Compute the relative rotation
  double dtime = cam0_curr_time - cam0_prev_time;
  Rodrigues(cam0_mean_ang_vel * dtime, cam0_R_p_c);
  Rodrigues(cam1_mean_ang_vel * dtime, cam1_R_p_c);
  cam0_R_p_c = cam0_R_p_c.t();
  cam1_R_p_c = cam1_R_p_c.t();

  // Delete the useless and used imu messages
  imu_data_buffer.erase(imu_data_buffer.begin(), end_iter);
}

void ImageProcessorCore::rescalePoints(
    vector<Point2f>& pts1, vector<Point2f>& pts2,
    float& scaling_factor) {

  scaling_factor = 0.0f;

  for (size_t i = 0; i < pts1.size(); ++i) {
    scaling_factor += sqrt(pts1[i].dot(pts1[i]));
    scaling_factor += sqrt(pts2[i].dot(pts2[i]));
  }

  scaling_factor = (pts1.size() + pts2.size()) / scaling_factor * sqrt(2.0f);

  for (size_t i = 0; i < pts1.size(); ++i) {
    pts1[i] *= scaling_factor;
    pts2[i] *= scaling_factor;
  }
}

void ImageProcessorCore::twoPointRansac(
    const vector<Point2f>& pts1, const vector<Point2f>& pts2,
    const Matx33f& R_p_c, const Vec4d& intrinsics,
    const std::string& distortion_model,
    const Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability,
    vector<int>& inlier_markers) {

  double norm_pixel_unit = 2.0 / (intrinsics[0] + intrinsics[1]);
  int iter_num = static_cast<int>(
      ceil(log(1 - success_probability) / log(1 - 0.7 * 0.7)));

  // Initially, mark all points as inliers
  inlier_markers.clear();
  inlier_markers.resize(pts1.size(), 1);

  // Undistort all the points
  vector<Point2f> pts1_undistorted(pts1.size());
  vector<Point2f> pts2_undistorted(pts2.size());
  undistortPoints(pts1, intrinsics, distortion_model, distortion_coeffs, pts1_undistorted);
  undistortPoints(pts2, intrinsics, distortion_model, distortion_coeffs, pts2_undistorted);

  // Compensate the points in the previous image with the relative rotation
  for (auto& pt : pts1_undistorted) {
    Vec3f pt_h(pt.x, pt.y, 1.0f);
    Vec3f pt_hc = R_p_c * pt_h;
    pt.x = pt_hc[0];
    pt.y = pt_hc[1];
  }

  // Normalize the points to gain numerical stability
  float scaling_factor = 0.0f;
  rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
  norm_pixel_unit *= scaling_factor;

  // Compute the difference between previous and current points
  vector<Point2d> pts_diff(pts1_undistorted.size());
  for (size_t i = 0; i < pts1_undistorted.size(); ++i)
    pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

  // Mark the point pairs with large difference directly
  double mean_pt_distance = 0.0;
  int raw_inlier_cntr = 0;
  for (size_t i = 0; i < pts_diff.size(); ++i) {
    double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
    if (distance > 50.0 * norm_pixel_unit) {
      inlier_markers[i] = 0;
    } else {
      mean_pt_distance += distance;
      ++raw_inlier_cntr;
    }
  }
  mean_pt_distance /= raw_inlier_cntr;

  // If the current number of inliers is less than 3, just mark all input as outliers
  if (raw_inlier_cntr < 3) {
    for (auto& marker : inlier_markers) marker = 0;
    return;
  }

  // Check for degenerated motion
  if (mean_pt_distance < norm_pixel_unit) {
    for (size_t i = 0; i < pts_diff.size(); ++i) {
      if (inlier_markers[i] == 0) continue;
      if (sqrt(pts_diff[i].dot(pts_diff[i])) > inlier_error * norm_pixel_unit)
        inlier_markers[i] = 0;
    }
    return;
  }

  // In the case of general motion, the RANSAC model can be applied
  Eigen::MatrixXd coeff_t(pts_diff.size(), 3);
  for (size_t i = 0; i < pts_diff.size(); ++i) {
    coeff_t(i, 0) = pts_diff[i].y;
    coeff_t(i, 1) = -pts_diff[i].x;
    coeff_t(i, 2) = pts1_undistorted[i].x * pts2_undistorted[i].y -
                    pts1_undistorted[i].y * pts2_undistorted[i].x;
  }

  vector<int> raw_inlier_idx;
  for (size_t i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] != 0)
      raw_inlier_idx.push_back(i);
  }
  if (raw_inlier_idx.size() < 2) {
    inlier_markers.assign(inlier_markers.size(), 0);
    return;
  }

  vector<int> best_inlier_set;
  double best_error = 1e10;
  std::mt19937 rng(std::random_device{}());

  for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx) {
    // Randomly select two point pairs
    std::uniform_int_distribution<int> dist_first(0, static_cast<int>(raw_inlier_idx.size()) - 1);
    std::uniform_int_distribution<int> dist_diff(1, static_cast<int>(raw_inlier_idx.size()) - 1);
    int select_idx1 = dist_first(rng);
    int select_idx_diff = dist_diff(rng);
    int select_idx2 = select_idx1 + select_idx_diff < static_cast<int>(raw_inlier_idx.size()) ?
                          select_idx1 + select_idx_diff :
                          select_idx1 + select_idx_diff - raw_inlier_idx.size();

    int pair_idx1 = raw_inlier_idx[select_idx1];
    int pair_idx2 = raw_inlier_idx[select_idx2];

    // Construct the model
    Eigen::Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
    Eigen::Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
    Eigen::Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
    vector<double> coeff_l1_norm(3);
    coeff_l1_norm[0] = coeff_tx.lpNorm<1>();
    coeff_l1_norm[1] = coeff_ty.lpNorm<1>();
    coeff_l1_norm[2] = coeff_tz.lpNorm<1>();
    int base_indicator = min_element(coeff_l1_norm.begin(), coeff_l1_norm.end()) - coeff_l1_norm.begin();

    Eigen::Vector3d model(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      Eigen::Matrix2d A;
      A << coeff_ty, coeff_tz;
      Eigen::Vector2d solution = A.inverse() * (-coeff_tx);
      model(0) = 1.0;
      model(1) = solution(0);
      model(2) = solution(1);
    } else if (base_indicator == 1) {
      Eigen::Matrix2d A;
      A << coeff_tx, coeff_tz;
      Eigen::Vector2d solution = A.inverse() * (-coeff_ty);
      model(0) = solution(0);
      model(1) = 1.0;
      model(2) = solution(1);
    } else {
      Eigen::Matrix2d A;
      A << coeff_tx, coeff_ty;
      Eigen::Vector2d solution = A.inverse() * (-coeff_tz);
      model(0) = solution(0);
      model(1) = solution(1);
      model(2) = 1.0;
    }

    // Find all the inliers among point pairs
    Eigen::VectorXd error = coeff_t * model;

    vector<int> inlier_set;
    for (int i = 0; i < error.rows(); ++i) {
      if (inlier_markers[i] == 0) continue;
      if (std::abs(error(i)) < inlier_error * norm_pixel_unit)
        inlier_set.push_back(i);
    }

    // If the number of inliers is small, the current model is probably wrong
    if (inlier_set.size() < 0.2 * pts1_undistorted.size())
      continue;

    // Refit the model using all of the possible inliers
    Eigen::VectorXd coeff_tx_better(inlier_set.size());
    Eigen::VectorXd coeff_ty_better(inlier_set.size());
    Eigen::VectorXd coeff_tz_better(inlier_set.size());
    for (size_t i = 0; i < inlier_set.size(); ++i) {
      coeff_tx_better(i) = coeff_t(inlier_set[i], 0);
      coeff_ty_better(i) = coeff_t(inlier_set[i], 1);
      coeff_tz_better(i) = coeff_t(inlier_set[i], 2);
    }

    Eigen::Vector3d model_better(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_ty_better, coeff_tz_better;
      Eigen::Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
      model_better(0) = 1.0;
      model_better(1) = solution(0);
      model_better(2) = solution(1);
    } else if (base_indicator == 1) {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_tz_better;
      Eigen::Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
      model_better(0) = solution(0);
      model_better(1) = 1.0;
      model_better(2) = solution(1);
    } else {
      Eigen::MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_ty_better;
      Eigen::Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_tz_better);
      model_better(0) = solution(0);
      model_better(1) = solution(1);
      model_better(2) = 1.0;
    }

    // Compute the error and update the best model if possible
    Eigen::VectorXd new_error = coeff_t * model_better;

    double this_error = 0.0;
    for (const auto& inlier_idx : inlier_set)
      this_error += std::abs(new_error(inlier_idx));
    this_error /= inlier_set.size();

    if (inlier_set.size() > best_inlier_set.size()) {
      best_error = this_error;
      best_inlier_set = inlier_set;
    }
  }

  // Fill in the markers
  inlier_markers.clear();
  inlier_markers.resize(pts1.size(), 0);
  for (const auto& inlier_idx : best_inlier_set)
    inlier_markers[inlier_idx] = 1;
}

}  // namespace core
}  // namespace msckf_vio
