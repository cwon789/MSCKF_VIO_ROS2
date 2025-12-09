
#ifndef MSCKF_VIO_IMAGE_PROCESSOR_H
#define MSCKF_VIO_IMAGE_PROCESSOR_H

#include <vector>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <msckf_vio/msg/camera_measurement.hpp>
#include <msckf_vio/msg/tracking_info.hpp>
#include <msckf_vio/core/image/image_processor_core.h>

namespace msckf_vio {

/**
 * @brief ImageProcessor ROS2 wrapper for feature detection and tracking
 * Inherits from both rclcpp::Node (ROS interface) and core::ImageProcessorCore (algorithm)
 */
class ImageProcessor : public rclcpp::Node, public core::ImageProcessorCore {
public:
  // Constructor
  explicit ImageProcessor(const rclcpp::NodeOptions& options);

  // Disable copy and assign constructors
  ImageProcessor(const ImageProcessor&) = delete;
  ImageProcessor operator=(const ImageProcessor&) = delete;

  // Destructor
  ~ImageProcessor();

  // Initialize the object
  bool initialize();

  using Ptr = std::shared_ptr<ImageProcessor>;
  using ConstPtr = std::shared_ptr<const ImageProcessor>;

private:

  /**
   * @brief loadParameters Load parameters from ROS parameter server
   */
  bool loadParameters();

  /**
   * @brief createRosIO Create ROS publishers and subscribers
   */
  bool createRosIO();

  /**
   * @brief stereoCallback Callback function for stereo images
   * @param cam0_img Left image
   * @param cam1_img Right image
   */
  void stereoCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr cam0_img,
      const sensor_msgs::msg::Image::ConstSharedPtr cam1_img);

  /**
   * @brief imuCallback Callback function for IMU messages
   * @param msg IMU message
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief publish Publish features and tracking info
   */
  void publish();

  /**
   * @brief drawFeaturesStereo Draw tracked and newly detected features
   */
  void drawFeaturesStereo();

  /**
   * @brief Helper template for declaring and getting ROS parameters
   */
  template<typename T>
  T declare_and_get(const std::string& name, const T& default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    T value = default_value;
    this->get_parameter(name, value);
    return value;
  }

  // Previous and current images (ROS bridge)
  cv_bridge::CvImageConstPtr cam0_prev_img_ptr;
  cv_bridge::CvImageConstPtr cam0_curr_img_ptr;
  cv_bridge::CvImageConstPtr cam1_curr_img_ptr;

  // IMU message buffer (ROS messages)
  std::vector<sensor_msgs::msg::Imu> imu_msg_buffer;

  // ROS subscribers and publishers
  message_filters::Subscriber<sensor_msgs::msg::Image> cam0_img_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> cam1_img_sub;
  std::shared_ptr<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>> stereo_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Publisher<msckf_vio::msg::CameraMeasurement>::SharedPtr feature_pub;
  rclcpp::Publisher<msckf_vio::msg::TrackingInfo>::SharedPtr tracking_info_pub;
  image_transport::Publisher debug_stereo_pub;
};

typedef ImageProcessor::Ptr ImageProcessorPtr;
typedef ImageProcessor::ConstPtr ImageProcessorConstPtr;

}  // end namespace msckf_vio

#endif
