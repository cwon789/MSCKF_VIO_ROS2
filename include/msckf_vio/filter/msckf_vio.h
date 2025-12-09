
#ifndef MSCKF_VIO_H
#define MSCKF_VIO_H

#include <map>
#include <set>
#include <vector>
#include <deque>
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

#include "msckf_vio/state/imu_state.h"
#include "msckf_vio/state/cam_state.h"
#include "msckf_vio/state/feature.hpp"
#include "msckf_vio/core/filter/msckf_filter_core.h"
#include <msckf_vio/msg/camera_measurement.hpp>

namespace msckf_vio {
/*
 * @brief MsckfVio ROS wrapper for MSCKF filter
 *    Implements the algorithm in
 *    Anatasios I. Mourikis, and Stergios I. Roumeliotis,
 *    "A Multi-State Constraint Kalman Filter for Vision-aided
 *    Inertial Navigation".
 */
class MsckfVio : public rclcpp::Node, public core::MsckfFilterCore {
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

    // Filter related functions (ROS-specific wrappers)
    // Propogate the state
    void batchImuProcessing(const double& time_bound);
    void addFeatureObservations(const CameraMeasurementConstPtr& msg);

    // Wrapper for pruneCamStateBuffer - finds and removes redundant states
    void pruneCamStateBuffer();
    // Reset the system online if the uncertainty is too large.
    void onlineReset();

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

    // Debug/local map visualization
    bool keep_feature_history_;
    size_t feature_history_max_points_;
    std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      feature_history_;

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
