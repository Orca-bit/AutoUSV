//
// Created by liuhao on 2022/4/10.
//

#include "pid_controller/controller_node.hpp"

ControllerNode::ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  const auto fvel_kf = declare_parameter("fwd_vel/kf").get<double>();
  const auto fvel_kp = declare_parameter("fwd_vel/kp").get<double>();
  const auto fvel_ki = declare_parameter("fwd_vel/ki").get<double>();
  const auto fvel_kd = declare_parameter("fwd_vel/kd").get<double>();
  const auto fvel_imax = declare_parameter("fwd_vel/imax").get<double>();
  const auto fvel_imin = declare_parameter("fwd_vel/imin").get<double>();
  const PidGain gain{fvel_kf, fvel_kp, fvel_ki, fvel_kd, fvel_imax, fvel_imin};

  const auto fwd_max_vel = declare_parameter("fwd_max_vel").get<double>();
  const auto fwd_max_force = declare_parameter("fwd_max_force").get<double>();
  const auto bck_max_vel = declare_parameter("bck_max_vel").get<double>();
  const auto bck_max_force = declare_parameter("bck_max_force").get<double>();

  core_ = std::make_unique<Controller>(
    gain, Limits{fwd_max_vel, fwd_max_force, bck_max_vel, bck_max_force});

  const auto current_state_topic = declare_parameter("current_state_topic").get<std::string>();
  const auto target_state_topic = declare_parameter("target_state_topic").get<std::string>();
  const auto control_pub_topic = declare_parameter("control_pub_topic").get<std::string>();
  const auto time = declare_parameter("cycle_time_ms").get<int64_t>();

  init(current_state_topic, target_state_topic, control_pub_topic, std::chrono::milliseconds{time});
}

void ControllerNode::init(
  std::string current_state_topic,
  std::string target_state_topic,
  std::string control_pub_topic,
  std::chrono::nanoseconds time)
{
  m_cycle_time_ = time;
  m_read_timer_ = create_wall_timer(m_cycle_time_, [this]() {
    try {
      publish_command();
    } catch (...) {
      on_error(std::current_exception());
    }
  });
  ctrl_pub_ =
    create_publisher<usv_msgs::msg::VehicleControlCommand>(control_pub_topic, rclcpp::QoS{10U});

  const auto cb1 = [this](usv_msgs::msg::VehicleKinematicState::SharedPtr ptr) {
    m_current_state_ = std::make_unique<usv_msgs::msg::VehicleKinematicState>(*ptr);
  };
  const auto cb2 = [this](usv_msgs::msg::VehicleKinematicState::SharedPtr ptr) {
    m_target_state_ = std::make_unique<usv_msgs::msg::VehicleKinematicState>(*ptr);
  };

  current_states_sub_ = create_subscription<usv_msgs::msg::VehicleKinematicState>(
    current_state_topic, rclcpp::QoS{10U}, cb1);
  target_sub_ = create_subscription<usv_msgs::msg::VehicleKinematicState>(
    target_state_topic, rclcpp::QoS{10U}, cb2);
}

void ControllerNode::publish_command()
{
  auto cmd = usv_msgs::msg::VehicleControlCommand{};
  if (m_current_state_ && m_target_state_) {
    auto force = core_->compute_fwd_force(
      m_target_state_->state.longitudinal_velocity_mps,
      m_current_state_->state.longitudinal_velocity_mps);
    cmd.left_cmd = force / 2.;
    cmd.right_cmd = force / 2.;
  }
  ctrl_pub_->publish(cmd);
}

void ControllerNode::on_error(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(eptr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "VehicleInterface: Unknown error!");
  }
}
