#include <rclcpp/rclcpp.hpp>
#include "snp_motion_planning/ndt_planning_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("ndt_planning_server");
  auto server = std::make_shared<NDTPlanningServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
