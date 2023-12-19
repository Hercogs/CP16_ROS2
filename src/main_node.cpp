#include "tortoisebot_waypoints/action_server_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<mylib::ActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}