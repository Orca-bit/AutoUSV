//
// Created by liuhao on 2022/4/10.
//

#include "pid_controller/controller_node.hpp"

int32_t main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto nd = std::make_shared<ControllerNode>("pid_controller", rclcpp::NodeOptions{});

  rclcpp::spin(nd);

  rclcpp::shutdown();

  return 0;
}