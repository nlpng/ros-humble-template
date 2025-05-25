#include "ros_template_node/template_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TemplateNode>();

  RCLCPP_INFO(node->get_logger(), "Starting template node...");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
