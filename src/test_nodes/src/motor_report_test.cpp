#include <rclcpp/rclcpp.hpp>

#include <usv_msgs/msg/motor_report1.hpp>

class MotorReport1TestNode : public rclcpp::Node
{
public:
  MotorReport1TestNode();
  void init();

private:
  rclcpp::Subscription<usv_msgs::msg::MotorReport1>::SharedPtr m_left_sub{nullptr};
  rclcpp::Subscription<usv_msgs::msg::MotorReport1>::SharedPtr m_right_sub{nullptr};
};

MotorReport1TestNode::MotorReport1TestNode() : Node("motor_report1_test")
{
  init();
}

void MotorReport1TestNode::init()
{
  auto cb = [this](usv_msgs::msg::MotorReport1::SharedPtr msg) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "direction(0: forward, 1: backward): " << msg->rotate_direction
                                             << "power: " << msg->motor_power);
  };
  m_left_sub =
    create_subscription<usv_msgs::msg::MotorReport1>("left_motor_report1", rclcpp::QoS{10U}, cb);
  m_right_sub =
    create_subscription<usv_msgs::msg::MotorReport1>("right_motor_report1", rclcpp::QoS{10U}, cb);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorReport1TestNode>());
  rclcpp::shutdown();
  return 0;
}