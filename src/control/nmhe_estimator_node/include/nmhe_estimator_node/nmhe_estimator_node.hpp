//
// Created by liuhao on 2022/4/12.
//

#ifndef NMHE_ESTIMATOR_NODE_NMHE_ESTIMATOR_NODE_HPP
#define NMHE_ESTIMATOR_NODE_NMHE_ESTIMATOR_NODE_HPP

#include "nmhe_estimator_node/visibility_control.hpp"
#include <nmhe_estimator/nmhe_estimator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <usv_msgs/msg/env_estimation.hpp>
#include <usv_msgs/msg/motor_report1.hpp>
#include <usv_msgs/msg/vehicle_kinematic_state.hpp>

namespace motion
{
namespace control
{
namespace nmhe_estimator_node
{

using MotorReport = usv_msgs::msg::MotorReport1;
using VehicleStates = usv_msgs::msg::VehicleKinematicState;
using EnvEstimation = usv_msgs::msg::EnvEstimation;
using Real = nmhe_estimator::Real;

constexpr auto Horizon = nmhe_estimator::N;

class NMHE_ESTIMATOR_NODES_PUBLIC NmheEstimatorNode : public ::rclcpp::Node
{
public:
  NmheEstimatorNode(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~NmheEstimatorNode() override = default;

private:
  NMHE_ESTIMATOR_NODES_LOCAL void init(
    const std::string & pub_topic,
    const std::string & left_motor_sub_topic,
    const std::string & right_motor_sub_topic,
    const std::string & states_sub_topic);
  NMHE_ESTIMATOR_NODES_LOCAL void on_state(const VehicleStates::SharedPtr & msg);
  NMHE_ESTIMATOR_NODES_LOCAL void on_left_motor(const MotorReport::SharedPtr& msg);
  NMHE_ESTIMATOR_NODES_LOCAL void on_right_motor(const MotorReport::SharedPtr& msg);
  NMHE_ESTIMATOR_NODES_LOCAL void try_compute();

  std::unique_ptr<nmhe_estimator::NmheEstimator> core_;

  rclcpp::Publisher<EnvEstimation>::SharedPtr pub_;
  rclcpp::Subscription<MotorReport>::SharedPtr left_motor_sub_;
  rclcpp::Subscription<MotorReport>::SharedPtr right_motor_sub_;
  rclcpp::Subscription<VehicleStates>::SharedPtr states_sub_;

  std::deque<VehicleStates> states_buffer_;
  std::deque<MotorReport> left_motor_buffer_;
  std::deque<MotorReport> right_motor_buffer_;
};

}  // namespace nmhe_estimator_node
}  // namespace control
}  // namespace motion

#endif  // NMHE_ESTIMATOR_NODE_NMHE_ESTIMATOR_NODE_HPP
