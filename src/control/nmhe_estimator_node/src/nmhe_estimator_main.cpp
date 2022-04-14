//
// Created by liuhao on 2022/4/12.
//

#include "nmhe_estimator_node/nmhe_estimator_node.hpp"

int32_t main(int32_t argc, char ** argv)
{
  rclcpp::init(argc, argv);
  using motion::control::nmhe_estimator_node::NmheEstimatorNode;
  const auto nd = std::make_shared<NmheEstimatorNode>("nmpc_controller", rclcpp::NodeOptions{});
  rclcpp::spin(nd);
  rclcpp::shutdown();
  return 0;
}