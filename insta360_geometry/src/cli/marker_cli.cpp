#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("marker_cli");
  auto pub = node->create_publisher<std_msgs::msg::String>(
    "/quaternion_visualizer/marker_command", 10);

  std::cout << "Commands: draw w x y z | delete w x y z | clear | quit\n";
  std::string line;
  while (rclcpp::ok() && std::getline(std::cin, line)) {
    if (line == "quit" || line == "exit") {
      break;
    }
    if (line.empty()) {
      continue;
    }
    std_msgs::msg::String msg;
    msg.data = line;
    pub->publish(msg);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
