# MSCKF\_VIO (ROS 2 Foxy, Ubuntu 20.04)

Stereo MSCKF VIO that takes synchronized stereo images + IMU and publishes real-time 6-DoF pose.  
Video: [demo](https://www.youtube.com/watch?v=jxfJFgzmNSw&t=3s) · Paper draft: [arXiv](https://arxiv.org/abs/1712.00036)

## Requirements
- ROS 2 Foxy on Ubuntu 20.04
- Build deps: Eigen3, OpenCV, Boost, PCL, SuiteSparse, tf2, image_transport, cv_bridge, message_filters, pcl_conversions, etc.
- SuiteSparse dev package (for `find_package(SuiteSparse)`):  
  `sudo apt-get install libsuitesparse-dev`

## Build & Source
```
cd ~/catkin_msckf
colcon build --packages-select msckf_vio --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
`colcon list` should show `msckf_vio (ros.ament_cmake)` after the build.

## Run (EuRoC / Fast flight datasets)
1) Download a dataset bag (e.g., EuRoC V1_01_easy) and convert to rosbag2 if needed.  
2) Launch (two nodes: `image_processor`, `vio`):
```
ros2 launch msckf_vio msckf_vio_euroc.launch.py
# or
ros2 launch msckf_vio msckf_vio_fla.launch.py
```
3) Play the bag in another terminal:
```
ros2 bag play V1_01_easy
```
4) RViz configs: `msckf_vio/rviz/rviz_euroc_config.rviz` or `msckf_vio/rviz/rviz_fla_config.rviz`.

> Tip: The filter uses the first ~200 IMU messages for bias/orientation init; keep the robot stationary at startup.

## Feature history (accumulate for local map visualization)
Adds previously initialized feature positions into the published `feature` PointCloud2 for visualization only (filter state unchanged).
- Defaults: disabled.
- Parameters:
  - `feature_history.enabled` (bool, default false)
  - `feature_history.max_points` (int, default 5000, ≤0 disables accumulation)
- Example:
```
ros2 launch msckf_vio msckf_vio_euroc.launch.py \
  feature_history.enabled:=true feature_history.max_points:=8000
```
History is cleared on reset/online-reset.

## Calibration
- Use Kalibr (or similar) for stereo+IMU calibration; match the YAML format under `config`.
- `camx/T_cam_imu`: IMU → camx transform; `cam1/T_cn_cnm1`: cam0 → cam1 transform.

## ROS Nodes (summary)
- `image_processor`
  - Sub: `imu` (`sensor_msgs/Imu`), `cam[x]_image` (`sensor_msgs/Image`)
  - Pub: `features` (`msckf_vio/CameraMeasurement`), `tracking_info`, `debug_stereo_img`
- `vio`
  - Sub: `imu`, `features`
  - Pub: `odom` (`nav_msgs/Odometry`), `feature` (`sensor_msgs/PointCloud2`), `path`, `pose_with_covariance`, optional TF
