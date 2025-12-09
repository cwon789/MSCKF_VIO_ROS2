/*
 * MSCKF Filter Core - ROS-Independent Algorithm Implementation
 * Pure algorithm without any ROS dependencies
 */

#ifndef MSCKF_VIO_FILTER_CORE_H
#define MSCKF_VIO_FILTER_CORE_H

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <msckf_vio/state/imu_state.h>
#include <msckf_vio/state/cam_state.h>
#include <msckf_vio/state/feature.hpp>

namespace msckf_vio {
namespace core {

/**
 * @brief State Server containing IMU state and camera states
 */
struct StateServer {
  IMUState imu_state;
  CamStateServer cam_states;

  // State covariance matrix
  Eigen::MatrixXd state_cov;
  Eigen::Matrix<double, 12, 12> continuous_noise_cov;
};

/**
 * @brief MSCKF Filter Core - Pure algorithm implementation
 * This class contains only the core algorithm logic without any ROS dependencies
 */
class MsckfFilterCore {
public:
  MsckfFilterCore() {}
  virtual ~MsckfFilterCore() {}

  /**
   * @brief Process IMU measurement
   * @param time Current time
   * @param m_gyro Measured angular velocity
   * @param m_acc Measured linear acceleration
   */
  void processModel(const double& time,
                    const Eigen::Vector3d& m_gyro,
                    const Eigen::Vector3d& m_acc);

  /**
   * @brief Predict new state
   * @param dt Time interval
   * @param gyro Angular velocity (bias removed)
   * @param acc Linear acceleration (bias removed)
   */
  void predictNewState(const double& dt,
                       const Eigen::Vector3d& gyro,
                       const Eigen::Vector3d& acc);

  /**
   * @brief State augmentation - add new camera state
   * @param time Current time
   */
  void stateAugmentation(const double& time);

  /**
   * @brief Compute measurement Jacobian for single camera-feature pair
   * @param cam_state_id Camera state ID
   * @param feature_id Feature ID
   * @param H_x Output Jacobian wrt camera state (4x6)
   * @param H_f Output Jacobian wrt feature (4x3)
   * @param r Output residual (4x1)
   */
  void measurementJacobian(
      const StateIDType& cam_state_id,
      const FeatureIDType& feature_id,
      Eigen::Matrix<double, 4, 6>& H_x,
      Eigen::Matrix<double, 4, 3>& H_f,
      Eigen::Vector4d& r);

  /**
   * @brief Compute feature Jacobian (projects onto nullspace)
   * @param feature_id Feature ID
   * @param cam_state_ids Camera state IDs
   * @param H_x Output Jacobian wrt state
   * @param r Output residual
   */
  void featureJacobian(
      const FeatureIDType& feature_id,
      const std::vector<StateIDType>& cam_state_ids,
      Eigen::MatrixXd& H_x,
      Eigen::VectorXd& r);

  /**
   * @brief Remove lost features and perform measurement update
   */
  void removeLostFeatures();

  /**
   * @brief Measurement update using computed Jacobian and residual
   * @param H Measurement Jacobian
   * @param r Residual
   */
  void measurementUpdate(const Eigen::MatrixXd& H,
                         const Eigen::VectorXd& r);

  /**
   * @brief Chi-square gating test
   * @param H Measurement Jacobian
   * @param r Residual
   * @param dof Degrees of freedom
   * @return true if test passed
   */
  bool gatingTest(const Eigen::MatrixXd& H,
                  const Eigen::VectorXd& r,
                  const int& dof);

  /**
   * @brief Find redundant camera states for marginalization
   * @param rm_cam_state_ids Output: camera state IDs to remove
   */
  void findRedundantCamStates(
      std::vector<StateIDType>& rm_cam_state_ids);

  /**
   * @brief Prune camera state buffer
   * @param rm_cam_state_ids Camera state IDs to remove
   */
  void pruneCamStateBuffer(
      const std::vector<StateIDType>& rm_cam_state_ids);

  /**
   * @brief Reset the filter to initial state
   */
  void onlineReset();

  // Accessors
  StateServer& getStateServer() { return state_server; }
  const StateServer& getStateServer() const { return state_server; }

  std::map<FeatureIDType, Feature>& getMapServer() { return map_server; }
  const std::map<FeatureIDType, Feature>& getMapServer() const { return map_server; }

protected:
  // State server
  StateServer state_server;

  // Map server - features
  std::map<FeatureIDType, Feature> map_server;

  // Chi-squared test table
  static std::map<int, double> chi_squared_test_table;

  // Maximum camera states to keep
  int max_cam_state_size = 20;

  // Position and rotation threshold for redundancy check
  double translation_threshold = 0.4;
  double rotation_threshold = 0.2618;  // 15 degrees
  double tracking_rate_threshold = 0.5;

  // Current tracking rate (updated by feature tracking)
  double tracking_rate = 0.0;
};

}  // namespace core
}  // namespace msckf_vio

#endif  // MSCKF_VIO_FILTER_CORE_H
