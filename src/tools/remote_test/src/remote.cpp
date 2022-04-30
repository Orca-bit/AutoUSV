//
// Created by liuhao on 22-4-30.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <usv_msgs/msg/vehicle_control_command.hpp>

using Cmd = usv_msgs::msg::VehicleControlCommand;


class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("remote_subscriber")
  {
    using std::placeholders::_1;
    subscription_ = this->create_subscription<Cmd>(
      "usv/basic_cmd", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const Cmd::SharedPtr msg) const
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "left cmd " << msg->left_cmd << ", right cmd " << msg->right_cmd);
  }
  rclcpp::Subscription<Cmd>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}