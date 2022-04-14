//
// Created by liuhao on 2022/4/12.
//

#include "nmhe_estimator_node/nmhe_estimator_node.hpp"

namespace motion
{
namespace control
{
namespace nmhe_estimator_node
{
NmheEstimatorNode::NmheEstimatorNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  using Config = nmhe_estimator::Config;
  using OptimizationConfig = nmhe_estimator::OptimizationConfig;
  using Weights = nmhe_estimator::Weights;
  using VehicleConfig = nmhe_estimator::VehicleConfig;
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("vehicle.cg_to_front_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_rear_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_thrusters_long_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_thrusters_lateral_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.front_corner_stiffness").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.rear_corner_stiffness").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.mass11_kg").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.mass22_kg").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.mass33_kgm2").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.damping11_kgm_s").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.damping22_kgm_s").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.damping33_kgm2_s").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.width_m").get<double>())};
  const Weights r{
    static_cast<Real>(declare_parameter("weights.r.u_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.r.v_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.r.r_weight").get<double>()),
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,
    0.};
  const Weights q{
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,
    static_cast<Real>(declare_parameter("weights.q.u_noise_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q.v_noise_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q.r_noise_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q.env_u_noise_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q.env_v_noise_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q.env_r_noise_weight").get<double>()),
  };
  const Weights q0{
    static_cast<Real>(declare_parameter("weights.q0.u_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q0.v_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q0.r_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q0.env_u_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q0.env_v_weight").get<double>()),
    static_cast<Real>(declare_parameter("weights.q0.env_r_weight").get<double>()),
    0.,
    0.,
    0.,
    0.,
    0.,
    0.};
  OptimizationConfig opt_cfg{q0, q, r};
  core_ = std::make_unique<nmhe_estimator::NmheEstimator>(Config{vehicle_param, opt_cfg});

  const auto pub_topic = declare_parameter("pub_topic").get<std::string>();
  const auto left_motor_sub_topic = declare_parameter("left_motor_sub_topic").get<std::string>();
  const auto right_motor_sub_topic = declare_parameter("right_motor_sub_topic").get<std::string>();
  const auto states_sub_topic = declare_parameter("states_sub_topic").get<std::string>();

  init(pub_topic, left_motor_sub_topic, right_motor_sub_topic, states_sub_topic);
}

void NmheEstimatorNode::init(
  const std::string & pub_topic,
  const std::string & left_motor_sub_topic,
  const std::string & right_motor_sub_topic,
  const std::string & states_sub_topic)
{
  if (pub_topic.empty()) {
    throw std::domain_error("Publish topic not set");
  }
  if (left_motor_sub_topic.empty()) {
    throw std::domain_error("left_motor_sub_topic not set");
  }
  if (right_motor_sub_topic.empty()) {
    throw std::domain_error("right_motor_sub_topic not set");
  }
  if (states_sub_topic.empty()) {
    throw std::domain_error("states_sub_topic not set");
  }

  // subs
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  states_sub_ = create_subscription<VehicleStates>(
    states_sub_topic,
    rclcpp::QoS{10U},
    [this](const VehicleStates::SharedPtr msg) { on_state(msg); },
    SubAllocT{});
  left_motor_sub_ = create_subscription<MotorReport>(
    left_motor_sub_topic,
    rclcpp::QoS{10U},
    [this](const MotorReport::SharedPtr msg) { on_left_motor(msg); },
    SubAllocT{});
  right_motor_sub_ = create_subscription<MotorReport>(
    right_motor_sub_topic,
    rclcpp::QoS{10U},
    [this](const MotorReport::SharedPtr msg) { on_right_motor(msg); },
    SubAllocT{});

  // pub
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  pub_ = create_publisher<EnvEstimation>(pub_topic, rclcpp::QoS{10U}, PubAllocT{});
}

void NmheEstimatorNode::on_state(const VehicleStates::SharedPtr & msg)
{
  states_buffer_.push_back(*msg);
  if (states_buffer_.size() > Horizon) {
    states_buffer_.pop_front();
  }
  try_compute();
}

void NmheEstimatorNode::on_left_motor(const MotorReport::SharedPtr & msg)
{
  left_motor_buffer_.push_back(*msg);
  if (left_motor_buffer_.size() > Horizon) {
    left_motor_buffer_.pop_front();
  }
  try_compute();
}

void NmheEstimatorNode::on_right_motor(const MotorReport::SharedPtr & msg)
{
  right_motor_buffer_.push_back(*msg);
  if (right_motor_buffer_.size() > Horizon) {
    right_motor_buffer_.pop_front();
  }
  try_compute();
}

void NmheEstimatorNode::try_compute()
{
  if (
    std::min(
      states_buffer_.size(), std::min(left_motor_buffer_.size(), right_motor_buffer_.size())) >=
    Horizon) {
    /// expensive copy, but want to solve the data race
    std::vector<VehicleStates> states_vec{states_buffer_.cbegin(), states_buffer_.cend()};
    std::vector<MotorReport> left_motor_vec{left_motor_buffer_.cbegin(), left_motor_buffer_.cend()};
    std::vector<MotorReport> right_motor_vec{
      right_motor_buffer_.cbegin(), right_motor_buffer_.cend()};
    auto res = core_->compute_env_estimation(states_vec, left_motor_vec, right_motor_vec);
    pub_->publish(res);
  }
}

}  // namespace nmhe_estimator_node
}  // namespace control
}  // namespace motion
