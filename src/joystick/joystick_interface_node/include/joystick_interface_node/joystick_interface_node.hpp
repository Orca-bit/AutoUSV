//
// Created by liuhao on 2021/11/14.
//

#ifndef JOYSTICK_INTERFACE_NODE_JOYSTICK_INTERFACE_NODE_HPP
#define JOYSTICK_INTERFACE_NODE_JOYSTICK_INTERFACE_NODE_HPP

#include <joystick_interface/joystick_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace joystick_interface_node {

using joystick_interface::AxisMap;
using joystick_interface::AxisScaleMap;
using joystick_interface::ButtonMap;

class JoystickInterfaceNode : public ::rclcpp::Node {
 public:
  explicit JoystickInterfaceNode(const rclcpp::NodeOptions& node_options);

 private:
  std::unique_ptr<joystick_interface::JoystickInterface> m_core;
  // void init(const std::string& control_command,
  //           const std::string& state_command_topic,
  //           const std::string& joy_topic,
  //           const bool& recordreplay_command_enabled, const AxisMap& axis_map,
  //           const AxisScaleMap& axis_scale_map,
  //           const AxisScaleMap& axis_offset_map, const ButtonMap& button_map);

  /// Callback for joystick subscription: compute control and state command and
  /// publish
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

  using RawControl = usv_msgs::msg::RawControlCommand;

  rclcpp::Publisher<RawControl>::SharedPtr m_raw_cmd_pub{};
  rclcpp::Publisher<usv_msgs::msg::VehicleStateCommand>::SharedPtr
      m_state_cmd_pub{};
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_recordreplay_cmd_pub{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub{nullptr};
};
}  // namespace joystick_interface_node

#endif  // JOYSTICK_INTERFACE_NODE_JOYSTICK_INTERFACE_NODE_HPP
