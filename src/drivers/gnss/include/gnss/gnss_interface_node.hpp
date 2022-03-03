//
// Created by liuhao on 2022/3/3.
//

#ifndef GNSS_GNSS_INTERFACE_NODE_HPP
#define GNSS_GNSS_INTERFACE_NODE_HPP

#include "gnss/gnss_interface.hpp"
#include "gnss/visibility_control.hpp"
#include <rclcpp/rclcpp.hpp>

namespace usv
{
namespace drivers
{
namespace gnss
{

class GNSS_INTERFACE_PUBLIC GnssInterfaceNode : public ::rclcpp::Node
{
public:
  GnssInterfaceNode(const std::string & node_name, const rclcpp::NodeOptions & options);
  void set_interface(std::unique_ptr<GnssInterface> && interface) noexcept;

private:
  void init(const std::string& topic_name, const std::chrono::nanoseconds & cycle_time);
  void read_and_pub();
  void on_error(std::exception_ptr eptr);
  rclcpp::Logger logger() const noexcept;

  std::unique_ptr<GnssInterface> m_interface;
  std::chrono::nanoseconds m_cycle_time{};

  rclcpp::TimerBase::SharedPtr m_read_timer{nullptr};
  rclcpp::Publisher<State>::SharedPtr m_state_pub{nullptr};

};

}  // namespace gnss
}  // namespace drivers
}  // namespace usv

#endif  // GNSS_GNSS_INTERFACE_NODE_HPP
