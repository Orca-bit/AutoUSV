//
// Created by liuhao on 2021/11/14.
//

#ifndef JOYSTICK_INTERFACE_NODE_JOYSTICK_INTERFACE_NODE_HPP
#define JOYSTICK_INTERFACE_NODE_JOYSTICK_INTERFACE_NODE_HPP

#include <joystick_interface/joystick_interface.hpp>
#include <mpark_variant_vendor/variant.hpp>
#include <rclcpp/rclcpp.hpp>

namespace joystick_interface_node
{

using joystick_interface::AxisMap;
using joystick_interface::AxisScaleMap;

class JoystickInterfaceNode : public ::rclcpp::Node
{
public:
  explicit JoystickInterfaceNode(const rclcpp::NodeOptions & node_options);

private:
  std::unique_ptr<joystick_interface::JoystickInterface> m_core;

  /// Callback for joystick subscription: compute control and state command and
  /// publish
  void on_joy(sensor_msgs::msg::Joy::SharedPtr msg);

  using BasicControl = usv_msgs::msg::VehicleControlCommand;
  using HighLevelControl = usv_msgs::msg::HighLevelControlCommand;

  template<typename T>
  using PubT = typename rclcpp::Publisher<T>::SharedPtr;
  using ControlPub = mpark::variant<PubT<BasicControl>, PubT<HighLevelControl>>;
  ControlPub m_cmd_pub{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub{nullptr};
};
}  // namespace joystick_interface_node

#endif  // JOYSTICK_INTERFACE_NODE_JOYSTICK_INTERFACE_NODE_HPP
