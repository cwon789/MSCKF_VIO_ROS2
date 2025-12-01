from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  robot = LaunchConfiguration("robot")
  fixed_frame_id = LaunchConfiguration("fixed_frame_id")
  calibration_file = LaunchConfiguration("calibration_file")
  use_sim_time = LaunchConfiguration("use_sim_time")

  default_calibration = PathJoinSubstitution(
      [FindPackageShare("msckf_vio"), "config", "camchain-imucam-fla.yaml"]
  )

  return LaunchDescription(
      [
          DeclareLaunchArgument("robot", default_value="fla"),
          DeclareLaunchArgument("fixed_frame_id", default_value="vision"),
          DeclareLaunchArgument("use_sim_time", default_value="true"),
          DeclareLaunchArgument(
              "calibration_file",
              default_value=default_calibration,
              description="Camera-IMU calibration for FLA dataset",
          ),
          Node(
              package="msckf_vio",
              executable="image_processor_node",
              namespace=robot,
              name="image_processor",
              parameters=[
                  calibration_file,
                  {
                      "use_sim_time": use_sim_time,
                      "grid_row": 4,
                      "grid_col": 5,
                      "grid_min_feature_num": 2,
                      "grid_max_feature_num": 3,
                      "pyramid_levels": 3,
                      "patch_size": 15,
                      "fast_threshold": 10,
                      "max_iteration": 30,
                      "track_precision": 0.01,
                      "ransac_threshold": 3.0,
                      "stereo_threshold": 5.0,
                  },
              ],
              remappings=[
                  ("imu", "sync/imu/imu"),
                  ("cam0_image", "sync/cam0/image_raw"),
                  ("cam1_image", "sync/cam1/image_raw"),
              ],
              output="screen",
          ),
          Node(
              package="msckf_vio",
              executable="msckf_vio_node",
              namespace=robot,
              name="vio",
              parameters=[
                  calibration_file,
                  {
                      "use_sim_time": use_sim_time,
                      "publish_tf": True,
                      "frame_rate": 40.0,
                      "fixed_frame_id": fixed_frame_id,
                      "child_frame_id": "odom",
                      "max_cam_state_size": 20,
                      "position_std_threshold": 8.0,
                      "rotation_threshold": 0.2618,
                      "translation_threshold": 0.4,
                      "tracking_rate_threshold": 0.5,
                      "feature.config.translation_threshold": -1.0,
                      "noise.gyro": 0.01,
                      "noise.acc": 0.1,
                      "noise.gyro_bias": 0.005,
                      "noise.acc_bias": 0.05,
                      "noise.feature": 0.03,
                      "initial_state.velocity.x": 0.0,
                      "initial_state.velocity.y": 0.0,
                      "initial_state.velocity.z": 0.0,
                      "initial_covariance.velocity": 0.25,
                      "initial_covariance.gyro_bias": 0.0001,
                      "initial_covariance.acc_bias": 0.01,
                      "initial_covariance.extrinsic_rotation_cov": 3.0462e-4,
                      "initial_covariance.extrinsic_translation_cov": 2.5e-5,
                  },
              ],
              remappings=[
                  ("imu", "sync/imu/imu"),
                  ("mocap_odom", "mocap/odom"),
              ],
              output="screen",
          ),
      ]
  )
