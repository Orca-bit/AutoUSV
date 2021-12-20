// Copyright AutoUSV
//

#include <memory>

#include "nmpc_controller_nodes/nmpc_controller_node.hpp"

int32_t main(int32_t argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using motion::control::nmpc_controller_nodes::NmpcControllerNode;
  const auto nd = std::make_shared<NmpcControllerNode>("nmpc_controller", "");

  rclcpp::spin(nd);

  rclcpp::shutdown();

  return 0;
}
