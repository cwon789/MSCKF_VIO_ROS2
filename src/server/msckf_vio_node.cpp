
#include <iostream>
#include <iomanip>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <stdexcept>

#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SPQRSupport>
#include <boost/math/distributions/chi_squared.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <msckf_vio/filter/msckf_vio.h>
#include <msckf_vio/utils/math_utils.hpp>
#include <msckf_vio/utils/utils.h>
#include <msckf_vio/utils/ros_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;
using namespace Eigen;
using namespace msckf_vio::ros_utils;

namespace msckf_vio{
// Note: Static member variables for IMUState, CAMState, and Feature
// are now initialized in src/core/state_initialization.cpp

// Removed: now defined in core
// map<int, double> MsckfVio::chi_squared_test_table;

MsckfVio::MsckfVio(const rclcpp::NodeOptions& options):
  rclcpp::Node("msckf_vio", options),
  is_gravity_set(false),
  is_first_img(true),
  keep_feature_history_(false),
  feature_history_max_points_(0) {
  tf_pub = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  if (!initialize()) {
    throw std::runtime_error("Failed to initialize MSCKF VIO");
  }
}

bool MsckfVio::loadParameters() {
  // Frame id
  fixed_frame_id = declare_and_get<string>("fixed_frame_id", "world");
  child_frame_id = declare_and_get<string>("child_frame_id", "robot");
  publish_tf = declare_and_get<bool>("publish_tf", true);
  frame_rate = declare_and_get<double>("frame_rate", 40.0);
  position_std_threshold = declare_and_get<double>("position_std_threshold", 8.0);

  rotation_threshold = declare_and_get<double>("rotation_threshold", 0.2618);
  translation_threshold = declare_and_get<double>("translation_threshold", 0.4);
  tracking_rate_threshold = declare_and_get<double>("tracking_rate_threshold", 0.5);

  // Feature optimization parameters
  Feature::optimization_config.translation_threshold =
      declare_and_get<double>("feature.config.translation_threshold", 0.2);

  // Noise related parameters
  IMUState::gyro_noise = declare_and_get<double>("noise.gyro", 0.001);
  IMUState::acc_noise = declare_and_get<double>("noise.acc", 0.01);
  IMUState::gyro_bias_noise = declare_and_get<double>("noise.gyro_bias", 0.001);
  IMUState::acc_bias_noise = declare_and_get<double>("noise.acc_bias", 0.01);
  Feature::observation_noise = declare_and_get<double>("noise.feature", 0.01);

  // Use variance instead of standard deviation.
  IMUState::gyro_noise *= IMUState::gyro_noise;
  IMUState::acc_noise *= IMUState::acc_noise;
  IMUState::gyro_bias_noise *= IMUState::gyro_bias_noise;
  IMUState::acc_bias_noise *= IMUState::acc_bias_noise;
  Feature::observation_noise *= Feature::observation_noise;

  // Set the initial IMU state.
  // The intial orientation and position will be set to the origin
  // implicitly. But the initial velocity and bias can be
  // set by parameters.
  // TODO: is it reasonable to set the initial bias to 0?
  state_server.imu_state.velocity(0) =
      declare_and_get<double>("initial_state.velocity.x", 0.0);
  state_server.imu_state.velocity(1) =
      declare_and_get<double>("initial_state.velocity.y", 0.0);
  state_server.imu_state.velocity(2) =
      declare_and_get<double>("initial_state.velocity.z", 0.0);

  // The initial covariance of orientation and position can be
  // set to 0. But for velocity, bias and extrinsic parameters,
  // there should be nontrivial uncertainty.
  double gyro_bias_cov, acc_bias_cov, velocity_cov;
  velocity_cov = declare_and_get<double>("initial_covariance.velocity", 0.25);
  gyro_bias_cov = declare_and_get<double>("initial_covariance.gyro_bias", 1e-4);
  acc_bias_cov = declare_and_get<double>("initial_covariance.acc_bias", 1e-2);

  double extrinsic_rotation_cov, extrinsic_translation_cov;
  extrinsic_rotation_cov =
      declare_and_get<double>("initial_covariance.extrinsic_rotation_cov", 3.0462e-4);
  extrinsic_translation_cov =
      declare_and_get<double>("initial_covariance.extrinsic_translation_cov", 1e-4);

  state_server.state_cov = MatrixXd::Zero(21, 21);
  for (int i = 3; i < 6; ++i)
    state_server.state_cov(i, i) = gyro_bias_cov;
  for (int i = 6; i < 9; ++i)
    state_server.state_cov(i, i) = velocity_cov;
  for (int i = 9; i < 12; ++i)
    state_server.state_cov(i, i) = acc_bias_cov;
  for (int i = 15; i < 18; ++i)
    state_server.state_cov(i, i) = extrinsic_rotation_cov;
  for (int i = 18; i < 21; ++i)
    state_server.state_cov(i, i) = extrinsic_translation_cov;

  // Transformation offsets between the frames involved.
  Isometry3d T_imu_cam0 = utils::getTransformEigen(*this, "cam0.T_cam_imu");
  Isometry3d T_cam0_imu = T_imu_cam0.inverse();

  state_server.imu_state.R_imu_cam0 = T_cam0_imu.linear().transpose();
  state_server.imu_state.t_cam0_imu = T_cam0_imu.translation();
  CAMState::T_cam0_cam1 =
    utils::getTransformEigen(*this, "cam1.T_cn_cnm1");
  IMUState::T_imu_body =
    utils::getTransformEigen(*this, "T_imu_body").inverse();

  // Maximum number of camera states to be stored
  max_cam_state_size = declare_and_get<int>("max_cam_state_size", 30);
  keep_feature_history_ =
      declare_and_get<bool>("feature_history.enabled", false);
  int history_points =
      declare_and_get<int>("feature_history.max_points", 5000);
  feature_history_max_points_ =
      history_points > 0 ? static_cast<size_t>(history_points) : 0;

  RCLCPP_INFO(get_logger(), "===========================================");
  RCLCPP_INFO(get_logger(), "fixed frame id: %s", fixed_frame_id.c_str());
  RCLCPP_INFO(get_logger(), "child frame id: %s", child_frame_id.c_str());
  RCLCPP_INFO(get_logger(), "publish tf: %d", publish_tf);
  RCLCPP_INFO(get_logger(), "frame rate: %f", frame_rate);
  RCLCPP_INFO(get_logger(), "position std threshold: %f", position_std_threshold);
  RCLCPP_INFO(get_logger(), "Keyframe rotation threshold: %f", rotation_threshold);
  RCLCPP_INFO(get_logger(), "Keyframe translation threshold: %f", translation_threshold);
  RCLCPP_INFO(get_logger(), "Keyframe tracking rate threshold: %f", tracking_rate_threshold);
  RCLCPP_INFO(get_logger(), "gyro noise: %.10f", IMUState::gyro_noise);
  RCLCPP_INFO(get_logger(), "gyro bias noise: %.10f", IMUState::gyro_bias_noise);
  RCLCPP_INFO(get_logger(), "acc noise: %.10f", IMUState::acc_noise);
  RCLCPP_INFO(get_logger(), "acc bias noise: %.10f", IMUState::acc_bias_noise);
  RCLCPP_INFO(get_logger(), "observation noise: %.10f", Feature::observation_noise);
  RCLCPP_INFO(get_logger(), "keep feature history: %d (max %zu points)",
      keep_feature_history_, feature_history_max_points_);
  RCLCPP_INFO(get_logger(), "initial velocity: %f, %f, %f",
      state_server.imu_state.velocity(0),
      state_server.imu_state.velocity(1),
      state_server.imu_state.velocity(2));
  RCLCPP_INFO(get_logger(), "initial gyro bias cov: %f", gyro_bias_cov);
  RCLCPP_INFO(get_logger(), "initial acc bias cov: %f", acc_bias_cov);
  RCLCPP_INFO(get_logger(), "initial velocity cov: %f", velocity_cov);
  RCLCPP_INFO(get_logger(), "initial extrinsic rotation cov: %f",
      extrinsic_rotation_cov);
  RCLCPP_INFO(get_logger(), "initial extrinsic translation cov: %f",
      extrinsic_translation_cov);

  cout << T_imu_cam0.linear() << endl;
  cout << T_imu_cam0.translation().transpose() << endl;

  RCLCPP_INFO(get_logger(), "max camera state #: %d", max_cam_state_size);
  RCLCPP_INFO(get_logger(), "===========================================");
  return true;
}

bool MsckfVio::createRosIO() {
  odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
  path_pub = create_publisher<nav_msgs::msg::Path>("path", 10);
  pose_cov_pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_covariance", 10);
  feature_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      "feature_point_cloud", rclcpp::SensorDataQoS());

  reset_srv = create_service<std_srvs::srv::Trigger>(
      "reset",
      std::bind(&MsckfVio::resetCallback, this,
        std::placeholders::_1, std::placeholders::_2));

  imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&MsckfVio::imuCallback, this, std::placeholders::_1));
  feature_sub = create_subscription<msckf_vio::msg::CameraMeasurement>(
      "features", rclcpp::SensorDataQoS().keep_last(40),
      std::bind(&MsckfVio::featureCallback, this, std::placeholders::_1));

  mocap_odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "mocap_odom", rclcpp::SystemDefaultsQoS(),
      std::bind(&MsckfVio::mocapOdomCallback, this, std::placeholders::_1));
  mocap_odom_pub = create_publisher<nav_msgs::msg::Odometry>("gt_odom", 10);

  return true;
}

bool MsckfVio::initialize() {
  if (!loadParameters()) return false;
  RCLCPP_INFO(get_logger(), "Finish loading ROS parameters...");

  // Initialize state server
  state_server.continuous_noise_cov =
    Matrix<double, 12, 12>::Zero();
  state_server.continuous_noise_cov.block<3, 3>(0, 0) =
    Matrix3d::Identity()*IMUState::gyro_noise;
  state_server.continuous_noise_cov.block<3, 3>(3, 3) =
    Matrix3d::Identity()*IMUState::gyro_bias_noise;
  state_server.continuous_noise_cov.block<3, 3>(6, 6) =
    Matrix3d::Identity()*IMUState::acc_noise;
  state_server.continuous_noise_cov.block<3, 3>(9, 9) =
    Matrix3d::Identity()*IMUState::acc_bias_noise;

  // Initialize the chi squared test table with confidence
  // level 0.95.
  for (int i = 1; i < 100; ++i) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_test_table[i] =
      boost::math::quantile(chi_squared_dist, 0.05);
  }

  if (!createRosIO()) return false;
  RCLCPP_INFO(get_logger(), "Finish creating ROS IO...");

  return true;
}

void MsckfVio::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {

  // IMU msgs are pushed backed into a buffer instead of
  // being processed immediately. The IMU msgs are processed
  // when the next image is available, in which way, we can
  // easily handle the transfer delay.
  imu_msg_buffer.push_back(*msg);

  if (!is_gravity_set) {
    if (imu_msg_buffer.size() < 200) return;
    //if (imu_msg_buffer.size() < 10) return;
    initializeGravityAndBias();
    is_gravity_set = true;
  }

  return;
}

void MsckfVio::initializeGravityAndBias() {

  // Initialize gravity and gyro bias.
  Vector3d sum_angular_vel = Vector3d::Zero();
  Vector3d sum_linear_acc = Vector3d::Zero();

  for (const auto& imu_msg : imu_msg_buffer) {
    Vector3d angular_vel = Vector3d::Zero();
    Vector3d linear_acc = Vector3d::Zero();

    vectorMsgToEigen(imu_msg.angular_velocity, angular_vel);
    vectorMsgToEigen(imu_msg.linear_acceleration, linear_acc);

    sum_angular_vel += angular_vel;
    sum_linear_acc += linear_acc;
  }

  state_server.imu_state.gyro_bias =
    sum_angular_vel / imu_msg_buffer.size();
  //IMUState::gravity =
  //  -sum_linear_acc / imu_msg_buffer.size();
  // This is the gravity in the IMU frame.
  Vector3d gravity_imu =
    sum_linear_acc / imu_msg_buffer.size();

  // Initialize the initial orientation, so that the estimation
  // is consistent with the inertial frame.
  double gravity_norm = gravity_imu.norm();
  IMUState::gravity = Vector3d(0.0, 0.0, -gravity_norm);

  Quaterniond q0_i_w = Quaterniond::FromTwoVectors(
    gravity_imu, -IMUState::gravity);
  state_server.imu_state.orientation =
    rotationToQuaternion(q0_i_w.toRotationMatrix().transpose());

  return;
}

void MsckfVio::resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  RCLCPP_WARN(get_logger(), "Start resetting msckf vio...");
  feature_sub.reset();
  imu_sub.reset();

  // Reset the IMU state.
  IMUState& imu_state = state_server.imu_state;
  imu_state.time = 0.0;
  imu_state.orientation = Vector4d(0.0, 0.0, 0.0, 1.0);
  imu_state.position = Vector3d::Zero();
  imu_state.velocity = Vector3d::Zero();
  imu_state.gyro_bias = Vector3d::Zero();
  imu_state.acc_bias = Vector3d::Zero();
  imu_state.orientation_null = Vector4d(0.0, 0.0, 0.0, 1.0);
  imu_state.position_null = Vector3d::Zero();
  imu_state.velocity_null = Vector3d::Zero();

  // Remove all existing camera states.
  state_server.cam_states.clear();

  // Reset the state covariance.
  double gyro_bias_cov, acc_bias_cov, velocity_cov;
  velocity_cov = declare_and_get<double>("initial_covariance.velocity", 0.25);
  gyro_bias_cov = declare_and_get<double>("initial_covariance.gyro_bias", 1e-4);
  acc_bias_cov = declare_and_get<double>("initial_covariance.acc_bias", 1e-2);

  double extrinsic_rotation_cov, extrinsic_translation_cov;
  extrinsic_rotation_cov =
      declare_and_get<double>("initial_covariance.extrinsic_rotation_cov", 3.0462e-4);
  extrinsic_translation_cov =
      declare_and_get<double>("initial_covariance.extrinsic_translation_cov", 1e-4);

  state_server.state_cov = MatrixXd::Zero(21, 21);
  for (int i = 3; i < 6; ++i)
    state_server.state_cov(i, i) = gyro_bias_cov;
  for (int i = 6; i < 9; ++i)
    state_server.state_cov(i, i) = velocity_cov;
  for (int i = 9; i < 12; ++i)
    state_server.state_cov(i, i) = acc_bias_cov;
  for (int i = 15; i < 18; ++i)
    state_server.state_cov(i, i) = extrinsic_rotation_cov;
  for (int i = 18; i < 21; ++i)
    state_server.state_cov(i, i) = extrinsic_translation_cov;

  // Clear all exsiting features in the map.
  map_server.clear();
  feature_history_.clear();

  // Clear the IMU msg buffer.
  imu_msg_buffer.clear();

  // Reset the starting flags.
  is_gravity_set = false;
  is_first_img = true;

  // Restart the subscribers.
  imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&MsckfVio::imuCallback, this, std::placeholders::_1));
  feature_sub = create_subscription<msckf_vio::msg::CameraMeasurement>(
      "features", rclcpp::SensorDataQoS().keep_last(40),
      std::bind(&MsckfVio::featureCallback, this, std::placeholders::_1));

  res->success = true;
  res->message = "MSCKF VIO reset";
  RCLCPP_WARN(get_logger(), "Resetting msckf vio completed...");
}

void MsckfVio::featureCallback(
    const msckf_vio::msg::CameraMeasurement::SharedPtr msg) {

  // Return if the gravity vector has not been set.
  if (!is_gravity_set) return;

  // Start the system if the first image is received.
  // The frame where the first image is received will be
  // the origin.
  if (is_first_img) {
    is_first_img = false;
    state_server.imu_state.time = toSec(msg->header.stamp);
  }

  static int critical_time_cntr = 0;
  double processing_start_time = this->now().seconds();

  // Propogate the IMU state.
  // that are received before the image msg.
  rclcpp::Time start_time = this->now();
  batchImuProcessing(toSec(msg->header.stamp));
  double imu_processing_time = (this->now()-start_time).seconds();

  // Augment the state vector.
  start_time = this->now();
  stateAugmentation(toSec(msg->header.stamp));
  double state_augmentation_time = (this->now()-start_time).seconds();

  // Add new observations for existing features or new
  // features in the map server.
  start_time = this->now();
  addFeatureObservations(msg);
  double add_observations_time = (this->now()-start_time).seconds();

  // Perform measurement update if necessary.
  start_time = this->now();
  removeLostFeatures();
  double remove_lost_features_time = (this->now()-start_time).seconds();

  start_time = this->now();
  pruneCamStateBuffer();
  double prune_cam_states_time = (this->now()-start_time).seconds();

  // Publish the odometry.
  start_time = this->now();
  publish(rclcpp::Time(msg->header.stamp));
  double publish_time = (this->now()-start_time).seconds();

  (void)imu_processing_time;
  (void)state_augmentation_time;
  (void)add_observations_time;
  (void)publish_time;

  // Reset the system if necessary.
  onlineReset();

  double processing_end_time = this->now().seconds();
  double processing_time =
    processing_end_time - processing_start_time;
  if (processing_time > 1.0/frame_rate) {
    ++critical_time_cntr;
    RCLCPP_INFO(get_logger(), "\033[1;31mTotal processing time %f/%d...\033[0m",
        processing_time, critical_time_cntr);
    //printf("IMU processing time: %f/%f\n",
    //    imu_processing_time, imu_processing_time/processing_time);
    //printf("State augmentation time: %f/%f\n",
    //    state_augmentation_time, state_augmentation_time/processing_time);
    //printf("Add observations time: %f/%f\n",
    //    add_observations_time, add_observations_time/processing_time);
    printf("Remove lost features time: %f/%f\n",
        remove_lost_features_time, remove_lost_features_time/processing_time);
    printf("Remove camera states time: %f/%f\n",
        prune_cam_states_time, prune_cam_states_time/processing_time);
    //printf("Publish time: %f/%f\n",
    //    publish_time, publish_time/processing_time);
  }

  return;
}

void MsckfVio::mocapOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  static bool first_mocap_odom_msg = true;

  if (first_mocap_odom_msg) {
    Quaterniond orientation;
    Vector3d translation;
    pointMsgToEigen(msg->pose.pose.position, translation);
    quaternionMsgToEigen(msg->pose.pose.orientation, orientation);
    mocap_initial_frame.linear() = orientation.toRotationMatrix();
    mocap_initial_frame.translation() = translation;
    first_mocap_odom_msg = false;
  }

  Quaterniond orientation;
  Vector3d translation;
  pointMsgToEigen(msg->pose.pose.position, translation);
  quaternionMsgToEigen(msg->pose.pose.orientation, orientation);

  Eigen::Isometry3d T_b_v_gt;
  T_b_v_gt.linear() = orientation.toRotationMatrix();
  T_b_v_gt.translation() = translation;
  Eigen::Isometry3d T_b_w_gt = mocap_initial_frame.inverse() * T_b_v_gt;

  if (publish_tf && tf_pub) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = fixed_frame_id;
    tf_msg.child_frame_id = child_frame_id + "_mocap";
    tf_msg.transform = toTransformMsg(T_b_w_gt);
    tf_pub->sendTransform(tf_msg);
  }

  nav_msgs::msg::Odometry mocap_odom_msg;
  mocap_odom_msg.header.stamp = msg->header.stamp;
  mocap_odom_msg.header.frame_id = fixed_frame_id;
  mocap_odom_msg.child_frame_id = child_frame_id + "_mocap";

  poseEigenToMsg(T_b_w_gt, mocap_odom_msg.pose.pose);

  if (mocap_odom_pub) {
    mocap_odom_pub->publish(mocap_odom_msg);
  }
}

void MsckfVio::batchImuProcessing(const double& time_bound) {
  // Counter how many IMU msgs in the buffer are used.
  int used_imu_msg_cntr = 0;

  for (const auto& imu_msg : imu_msg_buffer) {
    double imu_time = toSec(imu_msg.header.stamp);
    if (imu_time < state_server.imu_state.time) {
      ++used_imu_msg_cntr;
      continue;
    }
    if (imu_time > time_bound) break;

    // Convert the msgs.
    Vector3d m_gyro, m_acc;
    vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
    vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);

    // Execute process model.
    processModel(imu_time, m_gyro, m_acc);
    ++used_imu_msg_cntr;
  }

  // Set the state ID for the new IMU state.
  state_server.imu_state.id = IMUState::next_id++;

  // Remove all used IMU msgs.
  imu_msg_buffer.erase(imu_msg_buffer.begin(),
      imu_msg_buffer.begin()+used_imu_msg_cntr);

  return;
}




void MsckfVio::addFeatureObservations(
    const CameraMeasurementConstPtr& msg) {

  StateIDType state_id = state_server.imu_state.id;
  int curr_feature_num = map_server.size();
  int tracked_feature_num = 0;

  // Add new observations for existing features or new
  // features in the map server.
  for (const auto& feature : msg->features) {
    if (map_server.find(feature.id) == map_server.end()) {
      // This is a new feature.
      map_server[feature.id] = Feature(feature.id);
      map_server[feature.id].observations[state_id] =
        Vector4d(feature.u0, feature.v0,
            feature.u1, feature.v1);
    } else {
      // This is an old feature.
      map_server[feature.id].observations[state_id] =
        Vector4d(feature.u0, feature.v0,
            feature.u1, feature.v1);
      ++tracked_feature_num;
    }
  }

  tracking_rate =
    static_cast<double>(tracked_feature_num) /
    static_cast<double>(curr_feature_num);

  return;
}







void MsckfVio::pruneCamStateBuffer() {

  if (state_server.cam_states.size() < max_cam_state_size)
    return;

  // Find two camera states to be removed.
  vector<StateIDType> rm_cam_state_ids(0);
  findRedundantCamStates(rm_cam_state_ids);

  // Find the size of the Jacobian matrix.
  int jacobian_row_size = 0;
  for (auto& item : map_server) {
    auto& feature = item.second;
    // Check how many camera states to be removed are associated
    // with this feature.
    vector<StateIDType> involved_cam_state_ids(0);
    for (const auto& cam_id : rm_cam_state_ids) {
      if (feature.observations.find(cam_id) !=
          feature.observations.end())
        involved_cam_state_ids.push_back(cam_id);
    }

    if (involved_cam_state_ids.size() == 0) continue;
    if (involved_cam_state_ids.size() == 1) {
      feature.observations.erase(involved_cam_state_ids[0]);
      continue;
    }

    if (!feature.is_initialized) {
      // Check if the feature can be initialize.
      if (!feature.checkMotion(state_server.cam_states)) {
        // If the feature cannot be initialized, just remove
        // the observations associated with the camera states
        // to be removed.
        for (const auto& cam_id : involved_cam_state_ids)
          feature.observations.erase(cam_id);
        continue;
      } else {
        if(!feature.initializePosition(state_server.cam_states)) {
          for (const auto& cam_id : involved_cam_state_ids)
            feature.observations.erase(cam_id);
          continue;
        }
      }
    }

    jacobian_row_size += 4*involved_cam_state_ids.size() - 3;
  }

  //cout << "jacobian row #: " << jacobian_row_size << endl;

  // Compute the Jacobian and residual.
  MatrixXd H_x = MatrixXd::Zero(jacobian_row_size,
      21+6*state_server.cam_states.size());
  VectorXd r = VectorXd::Zero(jacobian_row_size);
  int stack_cntr = 0;

  for (auto& item : map_server) {
    auto& feature = item.second;
    // Check how many camera states to be removed are associated
    // with this feature.
    vector<StateIDType> involved_cam_state_ids(0);
    for (const auto& cam_id : rm_cam_state_ids) {
      if (feature.observations.find(cam_id) !=
          feature.observations.end())
        involved_cam_state_ids.push_back(cam_id);
    }

    if (involved_cam_state_ids.size() == 0) continue;

    MatrixXd H_xj;
    VectorXd r_j;
    featureJacobian(feature.id, involved_cam_state_ids, H_xj, r_j);

    if (gatingTest(H_xj, r_j, involved_cam_state_ids.size())) {
      H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
      r.segment(stack_cntr, r_j.rows()) = r_j;
      stack_cntr += H_xj.rows();
    }

    for (const auto& cam_id : involved_cam_state_ids)
      feature.observations.erase(cam_id);
  }

  H_x.conservativeResize(stack_cntr, H_x.cols());
  r.conservativeResize(stack_cntr);

  // Perform measurement update.
  measurementUpdate(H_x, r);

  for (const auto& cam_id : rm_cam_state_ids) {
    int cam_sequence = std::distance(state_server.cam_states.begin(),
        state_server.cam_states.find(cam_id));
    int cam_state_start = 21 + 6*cam_sequence;
    int cam_state_end = cam_state_start + 6;

    // Remove the corresponding rows and columns in the state
    // covariance matrix.
    if (cam_state_end < state_server.state_cov.rows()) {
      state_server.state_cov.block(cam_state_start, 0,
          state_server.state_cov.rows()-cam_state_end,
          state_server.state_cov.cols()) =
        state_server.state_cov.block(cam_state_end, 0,
            state_server.state_cov.rows()-cam_state_end,
            state_server.state_cov.cols());

      state_server.state_cov.block(0, cam_state_start,
          state_server.state_cov.rows(),
          state_server.state_cov.cols()-cam_state_end) =
        state_server.state_cov.block(0, cam_state_end,
            state_server.state_cov.rows(),
            state_server.state_cov.cols()-cam_state_end);

      state_server.state_cov.conservativeResize(
          state_server.state_cov.rows()-6, state_server.state_cov.cols()-6);
    } else {
      state_server.state_cov.conservativeResize(
          state_server.state_cov.rows()-6, state_server.state_cov.cols()-6);
    }

    // Remove this camera state in the state vector.
    state_server.cam_states.erase(cam_id);
  }

  return;
}

void MsckfVio::onlineReset() {

  // Never perform online reset if position std threshold
  // is non-positive.
  if (position_std_threshold <= 0) return;
  static long long int online_reset_counter = 0;

  // Check the uncertainty of positions to determine if
  // the system can be reset.
  double position_x_std = std::sqrt(state_server.state_cov(12, 12));
  double position_y_std = std::sqrt(state_server.state_cov(13, 13));
  double position_z_std = std::sqrt(state_server.state_cov(14, 14));

  if (position_x_std < position_std_threshold &&
      position_y_std < position_std_threshold &&
      position_z_std < position_std_threshold) return;

  RCLCPP_WARN(get_logger(), "Start %lld online reset procedure...",
      ++online_reset_counter);
  RCLCPP_INFO(get_logger(), "Stardard deviation in xyz: %f, %f, %f",
      position_x_std, position_y_std, position_z_std);

  // Remove all existing camera states.
  state_server.cam_states.clear();

  // Clear all exsiting features in the map.
  map_server.clear();
  feature_history_.clear();

  // Reset the state covariance.
  double gyro_bias_cov, acc_bias_cov, velocity_cov;
  velocity_cov = declare_and_get<double>("initial_covariance.velocity", 0.25);
  gyro_bias_cov = declare_and_get<double>("initial_covariance.gyro_bias", 1e-4);
  acc_bias_cov = declare_and_get<double>("initial_covariance.acc_bias", 1e-2);

  double extrinsic_rotation_cov, extrinsic_translation_cov;
  extrinsic_rotation_cov =
      declare_and_get<double>("initial_covariance.extrinsic_rotation_cov", 3.0462e-4);
  extrinsic_translation_cov =
      declare_and_get<double>("initial_covariance.extrinsic_translation_cov", 1e-4);

  state_server.state_cov = MatrixXd::Zero(21, 21);
  for (int i = 3; i < 6; ++i)
    state_server.state_cov(i, i) = gyro_bias_cov;
  for (int i = 6; i < 9; ++i)
    state_server.state_cov(i, i) = velocity_cov;
  for (int i = 9; i < 12; ++i)
    state_server.state_cov(i, i) = acc_bias_cov;
  for (int i = 15; i < 18; ++i)
    state_server.state_cov(i, i) = extrinsic_rotation_cov;
  for (int i = 18; i < 21; ++i)
    state_server.state_cov(i, i) = extrinsic_translation_cov;

  RCLCPP_WARN(get_logger(), "%lld online reset complete...", online_reset_counter);
  return;
}

void MsckfVio::publish(const rclcpp::Time& time) {

  // Convert the IMU frame to the body frame.
  const IMUState& imu_state = state_server.imu_state;
  Eigen::Isometry3d T_i_w = Eigen::Isometry3d::Identity();
  T_i_w.linear() = quaternionToRotation(
      imu_state.orientation).transpose();
  T_i_w.translation() = imu_state.position;

  Eigen::Isometry3d T_b_w = IMUState::T_imu_body * T_i_w *
    IMUState::T_imu_body.inverse();
  Eigen::Vector3d body_velocity =
    IMUState::T_imu_body.linear() * imu_state.velocity;

  // Publish tf
  if (publish_tf && tf_pub) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = time;
    transform_msg.header.frame_id = fixed_frame_id;
    transform_msg.child_frame_id = child_frame_id;
    transform_msg.transform = toTransformMsg(T_b_w);
    tf_pub->sendTransform(transform_msg);
  }

  // Publish the odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = fixed_frame_id;
  odom_msg.child_frame_id = child_frame_id;

  poseEigenToMsg(T_b_w, odom_msg.pose.pose);
  vectorEigenToMsg(body_velocity, odom_msg.twist.twist.linear);

  // Convert the covariance.
  Matrix3d P_oo = state_server.state_cov.block<3, 3>(0, 0);
  Matrix3d P_op = state_server.state_cov.block<3, 3>(0, 12);
  Matrix3d P_po = state_server.state_cov.block<3, 3>(12, 0);
  Matrix3d P_pp = state_server.state_cov.block<3, 3>(12, 12);
  Matrix<double, 6, 6> P_imu_pose = Matrix<double, 6, 6>::Zero();
  P_imu_pose << P_pp, P_po, P_op, P_oo;

  Matrix<double, 6, 6> H_pose = Matrix<double, 6, 6>::Zero();
  H_pose.block<3, 3>(0, 0) = IMUState::T_imu_body.linear();
  H_pose.block<3, 3>(3, 3) = IMUState::T_imu_body.linear();
  Matrix<double, 6, 6> P_body_pose = H_pose *
    P_imu_pose * H_pose.transpose();

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      odom_msg.pose.covariance[6*i+j] = P_body_pose(i, j);

  // Construct the covariance for the velocity.
  Matrix3d P_imu_vel = state_server.state_cov.block<3, 3>(6, 6);
  Matrix3d H_vel = IMUState::T_imu_body.linear();
  Matrix3d P_body_vel = H_vel * P_imu_vel * H_vel.transpose();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      odom_msg.twist.covariance[i*6+j] = P_body_vel(i, j);

  if (odom_pub) {
    odom_pub->publish(odom_msg);
  }

  if (pose_cov_pub) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = odom_msg.header;
    pose_cov_msg.pose = odom_msg.pose;
    pose_cov_pub->publish(pose_cov_msg);
  }

  if (path_pub) {
    path_msg_.header.stamp = time;
    path_msg_.header.frame_id = fixed_frame_id;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path_msg_.header;
    pose_stamped.pose = odom_msg.pose.pose;
    path_msg_.poses.push_back(pose_stamped);
    path_pub->publish(path_msg_);
  }

  // Publish the 3D positions of the features that
  // has been initialized.
  auto feature_msg_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  feature_msg_ptr->header.frame_id = fixed_frame_id;
  feature_msg_ptr->height = 1;
  for (const auto& item : map_server) {
    const auto& feature = item.second;
    if (feature.is_initialized) {
      Vector3d feature_position =
        IMUState::T_imu_body.linear() * feature.position;
      feature_msg_ptr->points.push_back(pcl::PointXYZ(
            feature_position(0), feature_position(1), feature_position(2)));
    }
  }
  if (keep_feature_history_) {
    for (const auto& position_world : feature_history_) {
      Vector3d feature_position =
        IMUState::T_imu_body.linear() * position_world;
      feature_msg_ptr->points.push_back(pcl::PointXYZ(
            feature_position(0), feature_position(1), feature_position(2)));
    }
  }
  feature_msg_ptr->width = feature_msg_ptr->points.size();

  if (feature_pub) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*feature_msg_ptr, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame_id;
    cloud_msg.header.stamp = time;
    feature_pub->publish(cloud_msg);
  }
}

} // namespace msckf_vio

RCLCPP_COMPONENTS_REGISTER_NODE(msckf_vio::MsckfVio)
