//
// Created by liuhao on 2022/4/10.
//

#ifndef PID_CONTROLLER_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_CONTROLLER_NODE_HPP

#include "pid_controller//controller.hpp"
#include <rclcpp/rclcpp.hpp>

#include <usv_msgs/msg/vehicle_control_command.hpp>
#include <usv_msgs/msg/vehicle_kinematic_state.hpp>


class ControllerNode : public ::rclcpp::Node
{
public:
  ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~ControllerNode() override = default;



private:
  // init topic name
  void init(
    std::string current_state_topic,
    std::string target_state_topic,
    std::string control_pub_topic,
    std::chrono::nanoseconds time);

  // publish command
  void publish_command();

  /// Handle exception thrown in main loop. Default behavior is to set NodeState to ERROR
  void on_error(std::exception_ptr eptr);

  // only calculate when both states are not nullptr
  std::unique_ptr<usv_msgs::msg::VehicleKinematicState> m_current_state_{nullptr};
  std::unique_ptr<usv_msgs::msg::VehicleKinematicState> m_target_state_{nullptr};

  // core compute
  std::unique_ptr<Controller> core_{nullptr};
  // subscribe target state
  rclcpp::Subscription<usv_msgs::msg::VehicleKinematicState>::SharedPtr target_sub_{nullptr};
  // subscribe current state
  rclcpp::Subscription<usv_msgs::msg::VehicleKinematicState>::SharedPtr current_states_sub_{nullptr};
  // publish command
  rclcpp::Publisher<usv_msgs::msg::VehicleControlCommand>::SharedPtr ctrl_pub_{nullptr};

  std::chrono::nanoseconds m_cycle_time_{};
  rclcpp::TimerBase::SharedPtr m_read_timer_{nullptr};
};

#endif  // PID_CONTROLLER_CONTROLLER_NODE_HPP
