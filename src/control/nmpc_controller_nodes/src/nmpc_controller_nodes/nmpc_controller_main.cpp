// Copyright AutoUSV
//

#include <memory>

#include "mpc_controller_nodes/mpc_controller_node.hpp"

int32_t main(int32_t argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using motion::control::mpc_controller_nodes::MpcControllerNode;
  const auto nd = std::make_shared<MpcControllerNode>("mpc_controller", "");

  rclcpp::spin(nd);

  rclcpp::shutdown();

  return 0;
}
