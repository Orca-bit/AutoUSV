//
// Created by liuhao on 2021/11/14.
//

#include "joystick_interface_node/joystick_interface_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace joystick_interface_node
{
using namespace joystick_interface;

JoystickInterfaceNode::JoystickInterfaceNode(const rclcpp::NodeOptions & node_options)
: Node{"joystick_interface_node", node_options}
{
  // topics
  const auto control_command = declare_parameter("control_command").get<std::string>();

  // maps
  const auto check_set = [this](auto & map, auto key, const std::string & param_name) {
    const auto param = declare_parameter(param_name);
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      using MapT = std::remove_reference_t<decltype(map)>;
      using ValT = typename MapT::mapped_type;
      const auto val_raw =
        param.get<std::conditional_t<std::is_floating_point<ValT>::value, double_t, int64_t>>();
      map[key] = static_cast<ValT>(val_raw);
    }
  };

  AxisMap axis_map{};
  check_set(axis_map, Axes::LEFT_CMD, "axes.left_cmd");
  check_set(axis_map, Axes::RIGHT_CMD, "axes.right_cmd");

  AxisScaleMap axis_scale_map{};
  check_set(axis_scale_map, Axes::LEFT_CMD, "axis_scale.left_cmd");
  check_set(axis_scale_map, Axes::RIGHT_CMD, "axis_scale.right_cmd");

  AxisScaleMap axis_offset_map{};
  check_set(axis_offset_map, Axes::LEFT_CMD, "axis_offset.left_cmd");
  check_set(axis_offset_map, Axes::RIGHT_CMD, "axis_offset.right_cmd");

  // TODO: now only basic control available
  if (control_command == "basic") {
    m_cmd_pub = create_publisher<BasicControl>(
      "basic_cmd", rclcpp::QoS{10U}.reliable().durability_volatile());
  } else if (control_command == "high_level") {
    // RCLCPP_WARN_STREAM(get_logger(), "High Level cmd not implemented yet");
    m_cmd_pub = create_publisher<HighLevelControl>(
      "high_level_cmd", rclcpp::QoS{10U}.reliable().durability_volatile());
  } else {
    throw std::domain_error{
      "JoystickInterfaceNode does not support " + control_command + "command control mode"};
  }

  m_joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
      on_joy(msg);
    });

  m_core = std::make_unique<JoystickInterface>(axis_map, axis_scale_map, axis_offset_map);
}

// callback func
void JoystickInterfaceNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Command publish
  const auto compute_publish_command = [this, &msg](auto && pub) -> void {
    using MessageT =
      typename std::decay_t<decltype(pub)>::element_type::MessageUniquePtr::element_type;
    const auto cmd = m_core->compute_command<MessageT>(*msg);
    // for Debug
    if (std::is_same<MessageT, BasicControl>::value) {
      RCLCPP_INFO_STREAM(
        get_logger(), "Left_cmd: " << cmd.left_cmd << " Right_cmd: " << cmd.right_cmd);
    }
    pub->publish(cmd);
  };
  mpark::visit(compute_publish_command, m_cmd_pub);
}
}  // namespace joystick_interface_node

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_interface_node::JoystickInterfaceNode)
