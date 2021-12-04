#include "vehicle_interface/vehicle_interface_node.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <common/types.hpp>
#include <time_utils/time_utils.hpp>

using usv::common::types::bool8_t;
using usv::common::types::float32_t;
using usv::common::types::float64_t;


namespace usv
{
namespace drivers
{
namespace vehicle_interface
{

////////////////////////////////////////////////////////////////////////////////
VehicleInterfaceNode::VehicleInterfaceNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node{node_name, options}
{
  // Helper functions
  const auto topic_num_matches_from_param = [this](const auto param) {
    const auto name_param = declare_parameter(param);
    return TopicNumMatches{name_param.template get<std::string>()};
  };
  const auto time = [this](const auto param_name) -> std::chrono::milliseconds {
    const auto count_ms = declare_parameter(param_name).template get<int64_t>();
    return std::chrono::milliseconds{count_ms};
  };

  set_interface(std::make_unique<PlatformInterface>(
    topic_num_matches_from_param("left_thruster_usb_name").topic,
    topic_num_matches_from_param("right_thruster_usb_name").topic));

  // Actually init
  init(topic_num_matches_from_param("control_command"), time("cycle_time_ms"));
}


void VehicleInterfaceNode::set_interface(std::unique_ptr<PlatformInterface> && interface) noexcept
{
  m_interface = std::forward<std::unique_ptr<PlatformInterface> &&>(interface);
  RCLCPP_INFO_STREAM(
    logger(),
    "left_thruster_usb_name: " << m_interface->get_left_usb_port_name()
                               << ", right_thruster_usb_name: "
                               << m_interface->get_right_usb_port_name());
}

rclcpp::Logger VehicleInterfaceNode::logger() const noexcept
{
  return get_logger();
}

template <typename T> void VehicleInterfaceNode::on_command_message(const T &) {}

template <>
void VehicleInterfaceNode::on_command_message(const usv_msgs::msg::HighLevelControlCommand & msg)
{
  /// not implemented
  if (!m_interface->send_control_command(msg)) {
    on_control_send_failure();
  }
}

////////////////////////////////////////////////////////////////////////////////
template <>
void VehicleInterfaceNode::on_command_message(const usv_msgs::msg::VehicleControlCommand & msg)
{
  const auto stamp = time_utils::from_message(msg.stamp);
  const auto dt = stamp - m_last_command_stamp;

  // Time should not go backwards
  if (dt < std::chrono::nanoseconds::zero()) {
    throw std::domain_error{"Vehicle interface command went backwards in time!"};
  }

  if (dt > std::chrono::nanoseconds::zero()) {
    m_last_command_stamp = stamp;
    // Send
    if (!m_interface->send_control_command(msg)) {
      on_control_send_failure();
    }
  } else {
    RCLCPP_WARN(logger(), "Vehicle interface time did not increase, skipping");
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::init(
  const TopicNumMatches & control_command, const std::chrono::nanoseconds & cycle_time)
{
  m_cycle_time = cycle_time;

  const auto cmd_callback = [this](auto t) -> auto
  {
    using Ptr = typename decltype(t)::SharedPtr;
    return [this](Ptr msg) -> void {
      try {
        on_command_message(*msg);
      } catch (...) {
        on_error(std::current_exception());
      }
    };
  };
  if (control_command.topic == "basic") {
    using BasicControlCommand = usv_msgs::msg::VehicleControlCommand;
    m_command_sub = create_subscription<BasicControlCommand>(
      "basic_cmd", rclcpp::QoS{10U}, cmd_callback(BasicControlCommand{}));
  } else if (control_command.topic == "high_level") {
    using HCC = usv_msgs::msg::HighLevelControlCommand;
    m_command_sub =
      create_subscription<HCC>("high_level_cmd", rclcpp::QoS{10U}, cmd_callback(HCC{}));
  } else {
    throw std::domain_error{"Vehicle interface must have exactly one command subscription"};
  }
  check_invariants();
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::check_invariants()
{
  if (decltype(m_cycle_time)::zero() >= m_cycle_time) {
    throw std::domain_error{"Cycle time must be positive"};
  }
  // Check command sub
  const auto ctrl_not_null =
    mpark::visit([](auto && sub) -> bool8_t { return static_cast<bool8_t>(sub); }, m_command_sub);
  if (!ctrl_not_null) {
    throw std::domain_error{"Vehicle interface must have exactly one command subscription"};
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_control_send_failure()
{
  throw std::runtime_error{"Sending control command failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_read_timeout()
{
  throw std::runtime_error{"Receiving data failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_error(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(eptr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), e.what());
  } catch (...) {
    RCLCPP_ERROR(logger(), "VehicleInterface: Unknown error!");
  }
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv
