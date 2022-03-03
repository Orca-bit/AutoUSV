//
// Created by liuhao on 2022/3/2.
//

#include "gnss/gnss_interface_node.hpp"

#include <utility>

namespace usv
{
namespace drivers
{
namespace gnss
{

GnssInterfaceNode::GnssInterfaceNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  const auto param = declare_parameter("gnss_usb_name").get<std::string>();
  set_interface(std::make_unique<GnssInterface>(param));
  const auto count_ms =
    std::chrono::milliseconds{declare_parameter("cycle_time_ms").get<int64_t>()};
  init("gnss_report", count_ms);
}

void GnssInterfaceNode::set_interface(std::unique_ptr<GnssInterface> && interface) noexcept
{
  m_interface = std::forward<std::unique_ptr<GnssInterface> &&>(interface);
  RCLCPP_INFO_STREAM(logger(), "gnss_usb_name: " << m_interface->get_port_name());
}

rclcpp::Logger GnssInterfaceNode::logger() const noexcept
{
  return get_logger();
}

void GnssInterfaceNode::init(
  const std::string & topic_name, const std::chrono::nanoseconds & cycle_time)
{
  m_cycle_time = cycle_time;
  m_read_timer = create_wall_timer(m_cycle_time, [this]() {
    try {
      read_and_pub();
    } catch (...) {
      on_error(std::current_exception());
    }
  });
  m_state_pub = create_publisher<State>(topic_name, rclcpp::QoS{10U});
}

void GnssInterfaceNode::read_and_pub()
{
  m_state_pub->publish(m_interface->work());
}

void GnssInterfaceNode::on_error(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(std::move(eptr));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), e.what());
  } catch (...) {
    RCLCPP_ERROR(logger(), "GnssInterface: Unknown error!");
  }
}

}  // namespace gnss
}  // namespace drivers
}  // namespace usv