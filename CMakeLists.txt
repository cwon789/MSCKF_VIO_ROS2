cmake_minimum_required(VERSION 3.5)
project(msckf_vio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/FeatureMeasurement.msg"
  "msg/CameraMeasurement.msg"
  "msg/TrackingInfo.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)

# Core library - ROS independent
add_library(msckf_vio_core SHARED
  src/core/state_initialization.cpp
  src/core/conversion_utils.cpp
  src/core/msckf_filter_core.cpp
  src/core/image_processor_core.cpp
)
target_link_libraries(msckf_vio_core
  ${OpenCV_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)
target_include_directories(msckf_vio_core PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)

# Server library - ROS dependent
add_library(msckf_vio_lib SHARED
  src/server/msckf_vio.cpp
  src/server/utils.cpp
)
target_link_libraries(msckf_vio_lib
  msckf_vio_core
  ${SUITESPARSE_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(msckf_vio_lib
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  image_transport
  cv_bridge
  message_filters
  pcl_conversions
  PCL
  OpenCV
  Eigen3
  Boost
  SuiteSparse
)

add_library(image_processor_lib SHARED
  src/server/image_processor.cpp
  src/server/utils.cpp
)
target_link_libraries(image_processor_lib
  msckf_vio_core
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(image_processor_lib
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  image_transport
  cv_bridge
  message_filters
  OpenCV
  Eigen3
  Boost
)

add_executable(msckf_vio_node src/server/msckf_vio.cpp src/server/utils.cpp)
target_link_libraries(msckf_vio_node msckf_vio_core ${SUITESPARSE_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(msckf_vio_node rclcpp rclcpp_components geometry_msgs nav_msgs sensor_msgs std_srvs tf2 tf2_ros image_transport cv_bridge message_filters pcl_conversions PCL OpenCV Eigen3 Boost SuiteSparse)

add_executable(image_processor_node src/server/image_processor.cpp src/server/utils.cpp)
target_link_libraries(image_processor_node msckf_vio_core ${OpenCV_LIBRARIES})
ament_target_dependencies(image_processor_node rclcpp rclcpp_components geometry_msgs sensor_msgs tf2 tf2_ros image_transport cv_bridge message_filters OpenCV Eigen3 Boost)

rclcpp_components_register_nodes(msckf_vio_lib "msckf_vio::MsckfVio")
rclcpp_components_register_nodes(image_processor_lib "msckf_vio::ImageProcessor")

rosidl_target_interfaces(msckf_vio_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(image_processor_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(msckf_vio_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(image_processor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  msckf_vio_core
  msckf_vio_lib
  image_processor_lib
  msckf_vio_node
  image_processor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  image_transport
  cv_bridge
  message_filters
  pcl_conversions
  PCL
  eigen3_cmake_module
  OpenCV
  Eigen3
  Boost
  SuiteSparse
  rosidl_default_runtime
)

ament_package()
