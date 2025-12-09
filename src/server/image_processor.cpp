/**
 * @file image_processor_node.cpp
 * @brief ROS2 wrapper for image processor (server layer only)
 */

#include <iostream>
#include <algorithm>
#include <stdexcept>

#include <sensor_msgs/image_encodings.hpp>
#include <msckf_vio/msg/camera_measurement.hpp>
#include <msckf_vio/msg/tracking_info.hpp>
#include <msckf_vio/image/image_processor.h>
#include <msckf_vio/utils/utils.h>
#include <msckf_vio/utils/ros_conversions.h>
#include <rmw/qos_profiles.h>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;
using namespace cv;
using namespace msckf_vio::ros_utils;
using std::placeholders::_1;
using std::placeholders::_2;

namespace msckf_vio {

ImageProcessor::ImageProcessor(const rclcpp::NodeOptions& options) :
  rclcpp::Node("image_processor", options),
  core::ImageProcessorCore() {
  // Initialize feature grids (inherited from core)
  prev_features_ptr.reset(new GridFeatures());
  curr_features_ptr.reset(new GridFeatures());

  if (!initialize()) {
    throw std::runtime_error("Failed to initialize Image Processor");
  }
}

ImageProcessor::~ImageProcessor() {
  destroyAllWindows();
}

bool ImageProcessor::loadParameters() {
  // Camera calibration parameters
  cam0_distortion_model = declare_and_get<string>("cam0.distortion_model", string("radtan"));
  cam1_distortion_model = declare_and_get<string>("cam1.distortion_model", string("radtan"));

  vector<int64_t> cam0_resolution_temp =
      declare_and_get<vector<int64_t>>("cam0.resolution", {});
  vector<int64_t> cam1_resolution_temp =
      declare_and_get<vector<int64_t>>("cam1.resolution", {});
  if (cam0_resolution_temp.size() != 2 || cam1_resolution_temp.size() != 2) {
    RCLCPP_ERROR(get_logger(), "Camera resolutions must have exactly 2 entries.");
    return false;
  }
  cam0_resolution[0] = static_cast<int>(cam0_resolution_temp[0]);
  cam0_resolution[1] = static_cast<int>(cam0_resolution_temp[1]);
  cam1_resolution[0] = static_cast<int>(cam1_resolution_temp[0]);
  cam1_resolution[1] = static_cast<int>(cam1_resolution_temp[1]);

  vector<double> cam0_intrinsics_temp =
      declare_and_get<vector<double>>("cam0.intrinsics", {});
  vector<double> cam1_intrinsics_temp =
      declare_and_get<vector<double>>("cam1.intrinsics", {});
  if (cam0_intrinsics_temp.size() != 4 || cam1_intrinsics_temp.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Camera intrinsics must have 4 values.");
    return false;
  }
  cam0_intrinsics = cv::Vec4d(cam0_intrinsics_temp[0],
      cam0_intrinsics_temp[1], cam0_intrinsics_temp[2],
      cam0_intrinsics_temp[3]);
  cam1_intrinsics = cv::Vec4d(cam1_intrinsics_temp[0],
      cam1_intrinsics_temp[1], cam1_intrinsics_temp[2],
      cam1_intrinsics_temp[3]);

  vector<double> cam0_distortion_coeffs_temp =
      declare_and_get<vector<double>>("cam0.distortion_coeffs", {});
  vector<double> cam1_distortion_coeffs_temp =
      declare_and_get<vector<double>>("cam1.distortion_coeffs", {});
  if (cam0_distortion_coeffs_temp.size() != 4 ||
      cam1_distortion_coeffs_temp.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Camera distortion coeffs must have 4 values.");
    return false;
  }
  cam0_distortion_coeffs = cv::Vec4d(cam0_distortion_coeffs_temp[0],
      cam0_distortion_coeffs_temp[1], cam0_distortion_coeffs_temp[2],
      cam0_distortion_coeffs_temp[3]);
  cam1_distortion_coeffs = cv::Vec4d(cam1_distortion_coeffs_temp[0],
      cam1_distortion_coeffs_temp[1], cam1_distortion_coeffs_temp[2],
      cam1_distortion_coeffs_temp[3]);

  cv::Mat T_imu_cam0 = utils::getTransformCV(*this, "cam0.T_cam_imu");
  cv::Matx33d R_imu_cam0(T_imu_cam0(cv::Rect(0, 0, 3, 3)));
  cv::Vec3d t_imu_cam0 = T_imu_cam0(cv::Rect(3, 0, 1, 3));
  R_cam0_imu = R_imu_cam0.t();
  t_cam0_imu = -R_imu_cam0.t() * t_imu_cam0;

  cv::Mat T_cam0_cam1 = utils::getTransformCV(*this, "cam1.T_cn_cnm1");
  cv::Mat T_imu_cam1 = T_cam0_cam1 * T_imu_cam0;
  cv::Matx33d R_imu_cam1(T_imu_cam1(cv::Rect(0, 0, 3, 3)));
  cv::Vec3d t_imu_cam1 = T_imu_cam1(cv::Rect(3, 0, 1, 3));
  R_cam1_imu = R_imu_cam1.t();
  t_cam1_imu = -R_imu_cam1.t() * t_imu_cam1;

  // Processor parameters
  processor_config.grid_row = declare_and_get<int>("grid_row", 4);
  processor_config.grid_col = declare_and_get<int>("grid_col", 4);
  processor_config.grid_min_feature_num =
      declare_and_get<int>("grid_min_feature_num", 2);
  processor_config.grid_max_feature_num =
      declare_and_get<int>("grid_max_feature_num", 4);
  processor_config.pyramid_levels =
      declare_and_get<int>("pyramid_levels", 3);
  processor_config.patch_size =
      declare_and_get<int>("patch_size", 31);
  processor_config.fast_threshold =
      declare_and_get<int>("fast_threshold", 20);
  processor_config.max_iteration =
      declare_and_get<int>("max_iteration", 30);
  processor_config.track_precision =
      declare_and_get<double>("track_precision", 0.01);
  processor_config.ransac_threshold =
      declare_and_get<double>("ransac_threshold", 3.0);
  processor_config.stereo_threshold =
      declare_and_get<double>("stereo_threshold", 3.0);

  RCLCPP_INFO(get_logger(), "===========================================");
  RCLCPP_INFO(get_logger(), "cam0_resolution: %d, %d",
      cam0_resolution[0], cam0_resolution[1]);
  RCLCPP_INFO(get_logger(), "cam0_intrinscs: %f, %f, %f, %f",
      cam0_intrinsics[0], cam0_intrinsics[1],
      cam0_intrinsics[2], cam0_intrinsics[3]);
  RCLCPP_INFO(get_logger(), "cam0_distortion_model: %s",
      cam0_distortion_model.c_str());
  RCLCPP_INFO(get_logger(), "cam0_distortion_coefficients: %f, %f, %f, %f",
      cam0_distortion_coeffs[0], cam0_distortion_coeffs[1],
      cam0_distortion_coeffs[2], cam0_distortion_coeffs[3]);

  RCLCPP_INFO(get_logger(), "cam1_resolution: %d, %d",
      cam1_resolution[0], cam1_resolution[1]);
  RCLCPP_INFO(get_logger(), "cam1_intrinscs: %f, %f, %f, %f",
      cam1_intrinsics[0], cam1_intrinsics[1],
      cam1_intrinsics[2], cam1_intrinsics[3]);
  RCLCPP_INFO(get_logger(), "cam1_distortion_model: %s",
      cam1_distortion_model.c_str());
  RCLCPP_INFO(get_logger(), "cam1_distortion_coefficients: %f, %f, %f, %f",
      cam1_distortion_coeffs[0], cam1_distortion_coeffs[1],
      cam1_distortion_coeffs[2], cam1_distortion_coeffs[3]);

  cout << R_imu_cam0 << endl;
  cout << t_imu_cam0.t() << endl;

  RCLCPP_INFO(get_logger(), "grid_row: %d", processor_config.grid_row);
  RCLCPP_INFO(get_logger(), "grid_col: %d", processor_config.grid_col);
  RCLCPP_INFO(get_logger(), "grid_min_feature_num: %d",
      processor_config.grid_min_feature_num);
  RCLCPP_INFO(get_logger(), "grid_max_feature_num: %d",
      processor_config.grid_max_feature_num);
  RCLCPP_INFO(get_logger(), "pyramid_levels: %d",
      processor_config.pyramid_levels);
  RCLCPP_INFO(get_logger(), "patch_size: %d",
      processor_config.patch_size);
  RCLCPP_INFO(get_logger(), "fast_threshold: %d",
      processor_config.fast_threshold);
  RCLCPP_INFO(get_logger(), "max_iteration: %d",
      processor_config.max_iteration);
  RCLCPP_INFO(get_logger(), "track_precision: %f",
      processor_config.track_precision);
  RCLCPP_INFO(get_logger(), "ransac_threshold: %f",
      processor_config.ransac_threshold);
  RCLCPP_INFO(get_logger(), "stereo_threshold: %f",
      processor_config.stereo_threshold);
  RCLCPP_INFO(get_logger(), "===========================================");
  return true;
}

bool ImageProcessor::createRosIO() {
  feature_pub = create_publisher<msckf_vio::msg::CameraMeasurement>(
      "features", rclcpp::SensorDataQoS().keep_last(3));
  tracking_info_pub = create_publisher<msckf_vio::msg::TrackingInfo>(
      "tracking_info", 1);
  debug_stereo_pub = image_transport::create_publisher(
      this, "debug_stereo_image");

  cam0_img_sub.subscribe(this, "cam0_image", rmw_qos_profile_sensor_data);
  cam1_img_sub.subscribe(this, "cam1_image", rmw_qos_profile_sensor_data);
  stereo_sub = std::make_shared<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(cam0_img_sub, cam1_img_sub, 10);
  stereo_sub->registerCallback(std::bind(&ImageProcessor::stereoCallback, this, _1, _2));
  imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(50),
      std::bind(&ImageProcessor::imuCallback, this, _1));

  return true;
}

bool ImageProcessor::initialize() {
  if (!loadParameters()) return false;
  RCLCPP_INFO(get_logger(), "Finish loading ROS parameters...");

  // Create feature detector
  detector_ptr = FastFeatureDetector::create(
      processor_config.fast_threshold);

  if (!createRosIO()) return false;
  RCLCPP_INFO(get_logger(), "Finish creating ROS IO...");

  return true;
}

void ImageProcessor::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr cam0_img,
    const sensor_msgs::msg::Image::ConstSharedPtr cam1_img) {

  // Get the current image (ROS bridge)
  cam0_curr_img_ptr = cv_bridge::toCvShare(cam0_img,
      sensor_msgs::image_encodings::MONO8);
  cam1_curr_img_ptr = cv_bridge::toCvShare(cam1_img,
      sensor_msgs::image_encodings::MONO8);

  // Copy to core data structures
  cam0_curr_img = cam0_curr_img_ptr->image.clone();
  cam1_curr_img = cam1_curr_img_ptr->image.clone();
  cam0_curr_time = toSec(cam0_img->header.stamp);

  // Build the image pyramids
  createImagePyramids();

  // Detect features in the first frame
  if (is_first_img) {
    initializeFirstFrame();
    is_first_img = false;

    // Draw results
    drawFeaturesStereo();
  } else {
    // Track the feature in the previous image
    trackFeatures();

    // Add new features into the current image
    addNewFeatures();

    // Prune excess features
    pruneGridFeatures();

    // Draw results
    drawFeaturesStereo();
  }

  // Publish features in the current image
  publish();

  // Update the previous image and previous features
  cam0_prev_img_ptr = cam0_curr_img_ptr;
  cam0_prev_img = cam0_curr_img.clone();
  cam0_prev_time = cam0_curr_time;
  prev_features_ptr = curr_features_ptr;
  std::swap(prev_cam0_pyramid_, curr_cam0_pyramid_);

  // Initialize the current features to empty vectors
  curr_features_ptr.reset(new GridFeatures());
  for (int code = 0; code <
      processor_config.grid_row*processor_config.grid_col; ++code) {
    (*curr_features_ptr)[code] = vector<FeatureMetaData>(0);
  }
}

void ImageProcessor::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Wait for the first image to be set
  if (is_first_img) return;

  // Store in ROS message buffer
  imu_msg_buffer.push_back(*msg);

  // Also store in core data structure for algorithm use
  ImuData imu_data;
  imu_data.time = toSec(msg->header.stamp);
  imu_data.angular_velocity = cv::Vec3f(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
  imu_data_buffer.push_back(imu_data);
}

void ImageProcessor::publish() {
  // Publish features
  msckf_vio::msg::CameraMeasurement feature_msg;
  feature_msg.header = cam0_curr_img_ptr->header;

  vector<FeatureIDType> curr_ids(0);
  vector<Point2f> curr_cam0_points(0);
  vector<Point2f> curr_cam1_points(0);

  for (const auto& grid_features : (*curr_features_ptr)) {
    for (const auto& feature : grid_features.second) {
      curr_ids.push_back(feature.id);
      curr_cam0_points.push_back(feature.cam0_point);
      curr_cam1_points.push_back(feature.cam1_point);
    }
  }

  vector<Point2f> curr_cam0_points_undistorted(0);
  vector<Point2f> curr_cam1_points_undistorted(0);

  undistortPoints(
      curr_cam0_points, cam0_intrinsics, cam0_distortion_model,
      cam0_distortion_coeffs, curr_cam0_points_undistorted);
  undistortPoints(
      curr_cam1_points, cam1_intrinsics, cam1_distortion_model,
      cam1_distortion_coeffs, curr_cam1_points_undistorted);

  for (size_t i = 0; i < curr_ids.size(); ++i) {
    msckf_vio::msg::FeatureMeasurement measurement;
    measurement.id = curr_ids[i];
    measurement.u0 = curr_cam0_points_undistorted[i].x;
    measurement.v0 = curr_cam0_points_undistorted[i].y;
    measurement.u1 = curr_cam1_points_undistorted[i].x;
    measurement.v1 = curr_cam1_points_undistorted[i].y;
    feature_msg.features.push_back(measurement);
  }

  feature_pub->publish(feature_msg);

  // Publish tracking info
  msckf_vio::msg::TrackingInfo tracking_info_msg;
  tracking_info_msg.header = cam0_curr_img_ptr->header;
  tracking_info_msg.before_tracking = before_tracking;
  tracking_info_msg.after_tracking = after_tracking;
  tracking_info_msg.after_matching = after_matching;
  tracking_info_msg.after_ransac = after_ransac;
  tracking_info_pub->publish(tracking_info_msg);
}

void ImageProcessor::drawFeaturesStereo() {
  if (debug_stereo_pub.getNumSubscribers() > 0) {
    // Colors for different features
    Scalar tracked(0, 255, 0);
    Scalar new_feature(0, 255, 255);

    static int grid_height =
      cam0_curr_img_ptr->image.rows / processor_config.grid_row;
    static int grid_width =
      cam0_curr_img_ptr->image.cols / processor_config.grid_col;

    // Create an output image
    int img_height = cam0_curr_img_ptr->image.rows;
    int img_width = cam0_curr_img_ptr->image.cols;
    Mat out_img(img_height, img_width*2, CV_8UC3);
    cvtColor(cam0_curr_img_ptr->image,
             out_img.colRange(0, img_width), CV_GRAY2RGB);
    cvtColor(cam1_curr_img_ptr->image,
             out_img.colRange(img_width, img_width*2), CV_GRAY2RGB);

    // Draw grids on the image
    for (int i = 1; i < processor_config.grid_row; ++i) {
      Point pt1(0, i*grid_height);
      Point pt2(img_width*2, i*grid_height);
      line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }
    for (int i = 1; i < processor_config.grid_col; ++i) {
      Point pt1(i*grid_width, 0);
      Point pt2(i*grid_width, img_height);
      line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }
    for (int i = 1; i < processor_config.grid_col; ++i) {
      Point pt1(i*grid_width+img_width, 0);
      Point pt2(i*grid_width+img_width, img_height);
      line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }

    // Collect features ids in the previous frame
    vector<FeatureIDType> prev_ids(0);
    for (const auto& grid_features : *prev_features_ptr)
      for (const auto& feature : grid_features.second)
        prev_ids.push_back(feature.id);

    // Collect feature points in the previous frame
    map<FeatureIDType, Point2f> prev_cam0_points;
    map<FeatureIDType, Point2f> prev_cam1_points;
    for (const auto& grid_features : *prev_features_ptr)
      for (const auto& feature : grid_features.second) {
        prev_cam0_points[feature.id] = feature.cam0_point;
        prev_cam1_points[feature.id] = feature.cam1_point;
      }

    // Collect feature points in the current frame
    map<FeatureIDType, Point2f> curr_cam0_points;
    map<FeatureIDType, Point2f> curr_cam1_points;
    for (const auto& grid_features : *curr_features_ptr)
      for (const auto& feature : grid_features.second) {
        curr_cam0_points[feature.id] = feature.cam0_point;
        curr_cam1_points[feature.id] = feature.cam1_point;
      }

    // Draw tracked features
    for (const auto& id : prev_ids) {
      if (prev_cam0_points.find(id) != prev_cam0_points.end() &&
          curr_cam0_points.find(id) != curr_cam0_points.end()) {
        cv::Point2f prev_pt0 = prev_cam0_points[id];
        cv::Point2f prev_pt1 = prev_cam1_points[id] + Point2f(img_width, 0.0);
        cv::Point2f curr_pt0 = curr_cam0_points[id];
        cv::Point2f curr_pt1 = curr_cam1_points[id] + Point2f(img_width, 0.0);

        circle(out_img, curr_pt0, 3, tracked, -1);
        circle(out_img, curr_pt1, 3, tracked, -1);
        line(out_img, prev_pt0, curr_pt0, tracked, 1);
        line(out_img, prev_pt1, curr_pt1, tracked, 1);

        prev_cam0_points.erase(id);
        prev_cam1_points.erase(id);
        curr_cam0_points.erase(id);
        curr_cam1_points.erase(id);
      }
    }

    // Draw new features
    for (const auto& new_cam0_point : curr_cam0_points) {
      cv::Point2f pt0 = new_cam0_point.second;
      cv::Point2f pt1 = curr_cam1_points[new_cam0_point.first] +
        Point2f(img_width, 0.0);

      circle(out_img, pt0, 3, new_feature, -1);
      circle(out_img, pt1, 3, new_feature, -1);
    }

    cv_bridge::CvImage debug_image(cam0_curr_img_ptr->header, "bgr8", out_img);
    debug_stereo_pub.publish(debug_image.toImageMsg());
  }
}

}  // end namespace msckf_vio

RCLCPP_COMPONENTS_REGISTER_NODE(msckf_vio::ImageProcessor)

// Main entry point
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<msckf_vio::ImageProcessor>(options);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
