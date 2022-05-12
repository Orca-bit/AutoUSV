//
// Created by liuhao on 2022/3/3.
//

#ifndef GNSS_GNSS_INTERFACE_NODE_HPP
#define GNSS_GNSS_INTERFACE_NODE_HPP

#include "gnss/gnss_interface.hpp"
#include "gnss/geo_pos_conv.hpp"
#include "gnss/visibility_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <experimental/optional>

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
  void pub_tf(const State & state);
  void on_error(std::exception_ptr eptr);
  rclcpp::Logger logger() const noexcept;

  std::unique_ptr<GnssInterface> m_interface;
  std::chrono::nanoseconds m_cycle_time{};

  rclcpp::TimerBase::SharedPtr m_read_timer{nullptr};
  rclcpp::Publisher<State>::SharedPtr m_state_pub{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_br_{nullptr};

  std::experimental::optional<std::chrono::system_clock::time_point> m_timer{};

};

}  // namespace gnss
}  // namespace drivers
}  // namespace usv

#endif  // GNSS_GNSS_INTERFACE_NODE_HPP
