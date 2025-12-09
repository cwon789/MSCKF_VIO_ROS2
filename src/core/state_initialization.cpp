/*
 * Core State Initialization
 * Static member initialization for state classes
 * ROS-independent
 */

#include <msckf_vio/state/imu_state.h>
#include <msckf_vio/state/cam_state.h>
#include <msckf_vio/state/feature.hpp>

using namespace Eigen;

namespace msckf_vio {

// Static member variables in IMUState class
StateIDType IMUState::next_id = 0;
double IMUState::gyro_noise = 0.001;
double IMUState::acc_noise = 0.01;
double IMUState::gyro_bias_noise = 0.001;
double IMUState::acc_bias_noise = 0.01;
Vector3d IMUState::gravity = Vector3d(0, 0, -GRAVITY_ACCELERATION);
Isometry3d IMUState::T_imu_body = Isometry3d::Identity();

// Static member variables in CAMState class
Isometry3d CAMState::T_cam0_cam1 = Isometry3d::Identity();

// Static member variables in Feature class
FeatureIDType Feature::next_id = 0;
double Feature::observation_noise = 0.01;
Feature::OptimizationConfig Feature::optimization_config;

} // namespace msckf_vio
