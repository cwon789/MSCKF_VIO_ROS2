#include <rclcpp/rclcpp.hpp>
#include <msckf_vio/msckf_vio.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<msckf_vio::MsckfVio>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
