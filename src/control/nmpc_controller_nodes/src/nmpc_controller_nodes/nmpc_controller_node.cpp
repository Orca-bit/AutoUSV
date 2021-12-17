// Copyright 2021 AutoUSV

#include "mpc_controller_nodes/mpc_controller_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace control
{
namespace mpc_controller_nodes
{
////////////////////////////////////////////////////////////////////////////////
MpcControllerNode::MpcControllerNode(const std::string & name, const std::string & ns)
: ControllerBaseNode{name, ns}
{
  using mpc_controller::LimitsConfig;
  const LimitsConfig limits{
    {static_cast<Real>(
       declare_parameter("controller.limits.min_longitudinal_velocity_mps").get<double>()),
     static_cast<Real>(
       declare_parameter("controller.limits.max_longitudinal_velocity_mps").get<double>())},
    {static_cast<Real>(
       declare_parameter("controller.limits.min_lateral_velocity_mps").get<double>()),
     static_cast<Real>(
       declare_parameter("controller.limits.max_lateral_velocity_mps").get<double>())},
    {static_cast<Real>(declare_parameter("controller.limits.min_acceleration_mps2").get<double>()),
     static_cast<Real>(declare_parameter("controller.limits.max_acceleration_mps2").get<double>())},
    {static_cast<Real>(declare_parameter("controller.limits.min_yaw_rate_rps").get<double>()),
     static_cast<Real>(declare_parameter("controller.limits.max_yaw_rate_rps").get<double>())},
    {static_cast<Real>(declare_parameter("controller.limits.min_jerk_mps3").get<double>()),
     static_cast<Real>(declare_parameter("controller.limits.max_jerk_mps3").get<double>())},
    {static_cast<Real>(
       declare_parameter("controller.limits.min_thruster_force_newton").get<double>()),
     static_cast<Real>(
       declare_parameter("controller.limits.max_thruster_force_newton").get<double>())},
  };
  using mpc_controller::VehicleConfig;
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
  using controller_common::ControlReference;
  using mpc_controller::BehaviorConfig;
  auto ref_type = ControlReference::SPATIAL;
  if (declare_parameter("controller.behavior.is_temporal_reference").get<bool>()) {
    ref_type = ControlReference::TEMPORAL;
  }
  const BehaviorConfig behavior{
    static_cast<Real>(declare_parameter("controller.behavior.stop_rate_mps2").get<double>()),
    std::chrono::milliseconds(declare_parameter("controller.behavior.time_step_ms").get<int64_t>()),
    ref_type};

  using mpc_controller::OptimizationConfig;
  using mpc_controller::StateWeight;
  const OptimizationConfig weights{
    StateWeight{
      static_cast<Real>(declare_parameter("controller.weights.nominal.pose_x").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.pose_y").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.heading").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.nominal.longitudinal_velocity").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.nominal.lateral_velocity").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.yaw_rate").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.acceleration").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.jerk").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.left_cmd").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.right_cmd").get<double>()),
    },
    StateWeight{
      static_cast<Real>(declare_parameter("controller.weights.terminal.pose_x").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.pose_y").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.heading").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.terminal.longitudinal_velocity").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.terminal.lateral_velocity").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.yaw_rate").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.terminal.acceleration").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.jerk").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.left_cmd").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.right_cmd").get<double>()),
    }};

  using mpc_controller::Interpolation;
  auto interpolation = Interpolation::NO;
  if (declare_parameter("controller.interpolation").get<bool>()) {
    interpolation = Interpolation::YES;
  }
  const auto sample_tolerance_ms =
    std::chrono::milliseconds(declare_parameter("controller.sample_tolerance_ms").get<int64_t>());

  const auto control_lookahead_ms =
    std::chrono::milliseconds(declare_parameter("controller.control_lookahead_ms").get<int64_t>());

  using mpc_controller::EnvironmentForces;


  auto controller = std::make_unique<mpc_controller::MpcController>(mpc_controller::Config{
    m_env_forces,
    limits,
    vehicle_param,
    behavior,
    weights,
    sample_tolerance_ms,
    control_lookahead_ms,
    interpolation});
  // I argue this is ok for the following reasons:
  // The parent class, ControllerBaseNode, has unique ownership of the controller, and the timer
  // only has a non-owning pointer. This is fine because the timer can never go out of scope before
  // the base class (and thus the owning pointer)
  const auto ctrl_ptr = controller.get();
  set_controller(std::move(controller));

  const auto debug_cycle_duration_param = declare_parameter("debug_trajectory_publish_period_ms");
  if (rclcpp::PARAMETER_INTEGER == debug_cycle_duration_param.get_type()) {
    const auto cycle_duration =
      std::chrono::milliseconds{debug_cycle_duration_param.get<std::int64_t>()};
    if (decltype(cycle_duration)::zero() != cycle_duration) {
      m_debug_traj_pub = create_publisher<usv_msgs::msg::Trajectory>(
        "mpc_debug_computed_trajectory", rclcpp::QoS{10LL});
      const auto debug_publish = [this, ctrl_ptr]() -> void {
        auto traj = ctrl_ptr->get_computed_trajectory();
        traj.header.frame_id = "map";
        m_debug_traj_pub->publish(traj);
      };
      m_debug_timer = create_wall_timer(cycle_duration, debug_publish);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
MpcControllerNode::MpcControllerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & command_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic,
  const std::string & static_tf_topic,
  const mpc_controller::Config & config)
: ControllerBaseNode{
    name,
    ns,
    command_topic,
    state_topic,
    tf_topic,
    trajectory_topic,
    diagnostic_topic,
    static_tf_topic}
{
  set_controller(std::make_unique<mpc_controller::MpcController>(config));
}
}  // namespace mpc_controller_nodes
}  // namespace control
}  // namespace motion
