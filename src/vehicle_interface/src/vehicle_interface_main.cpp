//
// Created by liuhao on 2021/12/3.
//
#include <vehicle_interface/vehicle_interface_node.hpp>

int32_t main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using usv::drivers::vehicle_interface::VehicleInterfaceNode;
  const auto nd =
    std::make_shared<VehicleInterfaceNode>("vehicle_interface", rclcpp::NodeOptions{});

  rclcpp::spin(nd);

  rclcpp::shutdown();

  return 0;
}