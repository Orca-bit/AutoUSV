// Copyright 2021 AutoUSV

#ifndef NMPC_CONTROLLER_NODES__NMPC_CONTROLLER_NODE_HPP_
#define NMPC_CONTROLLER_NODES__NMPC_CONTROLLER_NODE_HPP_

#include <string>

#include <controller_common_nodes/controller_base_node.hpp>
#include <nmpc_controller/nmpc_controller.hpp>
#include <nmpc_controller_nodes/visibility_control.hpp>

namespace motion
{
namespace control
{
namespace nmpc_controller_nodes
{

using nmpc_controller::Real;

class NMPC_CONTROLLER_NODES_PUBLIC NmpcControllerNode
: public controller_common_nodes::ControllerBaseNode
{
public:
  /// Parameter file constructor
  NmpcControllerNode(const std::string & name, const std::string & ns);
  /// Explicit constructor
  NmpcControllerNode(
    const std::string & name,
    const std::string & ns,
    const std::string & command_topic,
    const std::string & state_topic,
    const std::string & tf_topic,
    const std::string & trajectory_topic,
    const std::string & diagnostic_topic,
    const std::string & env_forces_topic,
    const std::string & static_tf_topic,
    const nmpc_controller::Config & config);

private:
  rclcpp::Publisher<usv_msgs::msg::Trajectory>::SharedPtr m_debug_traj_pub{};
  rclcpp::TimerBase::SharedPtr m_debug_timer{};
};  // class NmpcControllerNode
}  // namespace nmpc_controller_nodes
}  // namespace control
}  // namespace motion

#endif  // NMPC_CONTROLLER_NODES__NMPC_CONTROLLER_NODE_HPP_
