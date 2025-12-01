/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_H
#define MSCKF_VIO_H

#include <map>
#include <set>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "imu_state.h"
#include "cam_state.h"
#include "feature.hpp"
#include <msckf_vio/msg/camera_measurement.hpp>

namespace msckf_vio {
/*
 * @brief MsckfVio Implements the algorithm in
 *    Anatasios I. Mourikis, and Stergios I. Roumeliotis,
 *    "A Multi-State Constraint Kalman Filter for Vision-aided
 *    Inertial Navigation".
 */
class MsckfVio : public rclcpp::Node {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    explicit MsckfVio(const rclcpp::NodeOptions& options);
    // Disable copy and assign constructor
    MsckfVio(const MsckfVio&) = delete;
    MsckfVio operator=(const MsckfVio&) = delete;

    // Destructor
    ~MsckfVio() {}

    /*
     * @brief initialize Initialize the VIO.
     */
    bool initialize();

    /*
     * @brief reset Resets the VIO to initial status.
     */
    void reset();

  using Ptr = std::shared_ptr<MsckfVio>;
  using ConstPtr = std::shared_ptr<const MsckfVio>;
    using CameraMeasurementConstPtr = msckf_vio::msg::CameraMeasurement::ConstSharedPtr;

  private:
    /*
     * @brief StateServer Store one IMU states and several
     *    camera states for constructing measurement
     *    model.
     */
    struct StateServer {
      IMUState imu_state;
      CamStateServer cam_states;

      // State covariance matrix
      Eigen::MatrixXd state_cov;
      Eigen::Matrix<double, 12, 12> continuous_noise_cov;
    };


    /*
     * @brief loadParameters
     *    Load parameters from the parameter server.
     */
    bool loadParameters();

    /*
     * @brief createRosIO
     *    Create ros publisher and subscirbers.
     */
    bool createRosIO();

    /*
     * @brief imuCallback
     *    Callback function for the imu message.
     * @param msg IMU msg.
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /*
     * @brief featureCallback
     *    Callback function for feature measurements.
     * @param msg Stereo feature measurements.
     */
    void featureCallback(
      const msckf_vio::msg::CameraMeasurement::SharedPtr msg);

    /*
     * @brief publish Publish the results of VIO.
     * @param time The time stamp of output msgs.
     */
    void publish(const rclcpp::Time& time);

    /*
     * @brief initializegravityAndBias
     *    Initialize the IMU bias and initial orientation
     *    based on the first few IMU readings.
     */
    void initializeGravityAndBias();

    /*
     * @biref resetCallback
     *    Callback function for the reset service.
     *    Note that this is NOT anytime-reset. This function should
     *    only be called before the sensor suite starts moving.
     *    e.g. while the robot is still on the ground.
     */
    void resetCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // Filter related functions
    // Propogate the state
    void batchImuProcessing(
        const double& time_bound);
    void processModel(const double& time,
        const Eigen::Vector3d& m_gyro,
        const Eigen::Vector3d& m_acc);
    void predictNewState(const double& dt,
        const Eigen::Vector3d& gyro,
        const Eigen::Vector3d& acc);

    // Measurement update
    void stateAugmentation(const double& time);
    void addFeatureObservations(const CameraMeasurementConstPtr& msg);
    // This function is used to compute the measurement Jacobian
    // for a single feature observed at a single camera frame.
    void measurementJacobian(const StateIDType& cam_state_id,
        const FeatureIDType& feature_id,
        Eigen::Matrix<double, 4, 6>& H_x,
        Eigen::Matrix<double, 4, 3>& H_f,
        Eigen::Vector4d& r);
    // This function computes the Jacobian of all measurements viewed
    // in the given camera states of this feature.
    void featureJacobian(const FeatureIDType& feature_id,
        const std::vector<StateIDType>& cam_state_ids,
        Eigen::MatrixXd& H_x, Eigen::VectorXd& r);
    void measurementUpdate(const Eigen::MatrixXd& H,
        const Eigen::VectorXd& r);
    bool gatingTest(const Eigen::MatrixXd& H,
        const Eigen::VectorXd&r, const int& dof);
    void removeLostFeatures();
    void findRedundantCamStates(
        std::vector<StateIDType>& rm_cam_state_ids);
    void pruneCamStateBuffer();
    // Reset the system online if the uncertainty is too large.
    void onlineReset();

    // Chi squared test table.
    static std::map<int, double> chi_squared_test_table;

    // State vector
    StateServer state_server;
    // Maximum number of camera states
    int max_cam_state_size;

    // Features used
    MapServer map_server;

    // IMU data buffer
    // This is buffer is used to handle the unsynchronization or
    // transfer delay between IMU and Image messages.
    std::vector<sensor_msgs::msg::Imu> imu_msg_buffer;

    // Indicate if the gravity vector is set.
    bool is_gravity_set;

    // Indicate if the received image is the first one. The
    // system will start after receiving the first image.
    bool is_first_img;

    // The position uncertainty threshold is used to determine
    // when to reset the system online. Otherwise, the ever-
    // increaseing uncertainty will make the estimation unstable.
    // Note this online reset will be some dead-reckoning.
    // Set this threshold to nonpositive to disable online reset.
    double position_std_threshold;

    // Tracking rate
    double tracking_rate;

    // Threshold for determine keyframes
    double translation_threshold;
    double rotation_threshold;
    double tracking_rate_threshold;

    // Ros node handle

    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<msckf_vio::msg::CameraMeasurement>::SharedPtr feature_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr feature_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;

    // Frame id
    std::string fixed_frame_id;
    std::string child_frame_id;

    // Whether to publish tf or not.
    bool publish_tf;

    // Framte rate of the stereo images. This variable is
    // only used to determine the timing threshold of
    // each iteration of the filter.
    double frame_rate;

    // Debugging variables and functions
    void mocapOdomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mocap_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mocap_odom_pub;
    geometry_msgs::msg::TransformStamped raw_mocap_odom_msg;
    Eigen::Isometry3d mocap_initial_frame;

    nav_msgs::msg::Path path_msg_;

    template<typename T>
    T declare_and_get(const std::string &name, const T &default_value) {
      if (!this->has_parameter(name)) {
        this->declare_parameter<T>(name, default_value);
      }
      T value = default_value;
      this->get_parameter(name, value);
      return value;
    }
};

using MsckfVioPtr = std::shared_ptr<MsckfVio>;
using MsckfVioConstPtr = std::shared_ptr<const MsckfVio>;

} // namespace msckf_vio

#endif
