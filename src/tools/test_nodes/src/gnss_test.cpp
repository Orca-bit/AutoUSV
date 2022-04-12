//
// Created by liuhao on 2022/3/5.
//

#include <rclcpp/rclcpp.hpp>

#include <usv_msgs/msg/vehicle_kinematic_state.hpp>

using State = usv_msgs::msg::VehicleKinematicState;

class GnssTestNode : public ::rclcpp::Node
{
public:
  GnssTestNode();
  void init();

private:
  rclcpp::Subscription<State>::SharedPtr m_sub{nullptr};
};

GnssTestNode::GnssTestNode() : Node("gnss_test")
{
  init();
}

void GnssTestNode::init()
{
  auto cb = [this](State::SharedPtr msg) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "u: " << msg->state.longitudinal_velocity_mps << "v: " << msg->state.lateral_velocity_mps
            << "r: " << msg->state.heading_rate_rps);
  };
  m_sub = create_subscription<State>("gnss_report", rclcpp::QoS{10U}, cb);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssTestNode>());
  rclcpp::shutdown();
  return 0;
}
