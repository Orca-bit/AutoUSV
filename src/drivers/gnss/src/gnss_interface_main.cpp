//
// Created by liuhao on 2022/3/3.
//

#include "gnss/gnss_interface_node.hpp"

int32_t main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using usv::drivers::gnss::GnssInterfaceNode;
  const auto nd =
    std::make_shared<GnssInterfaceNode>("gnss_interface", rclcpp::NodeOptions{});

  rclcpp::spin(nd);

  rclcpp::shutdown();

  return 0;
}
