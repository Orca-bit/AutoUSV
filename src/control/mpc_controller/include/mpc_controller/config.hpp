#ifndef MPC_CONTROLLER__CONFIG_HPP_
#define MPC_CONTROLLER__CONFIG_HPP_

#include <controller_common/controller_base.hpp>
#include <motion_common/config.hpp>
#include <mpc_controller/visibility_control.hpp>

#define MPC_CONTROLLER_COPY_MOVE_ASSIGNABLE(Class) \
  Class(const Class &) = default;                  \
  Class(Class &&) = default;                       \
  Class & operator=(const Class &) = default;      \
  Class & operator=(Class &&) = default;           \
  ~Class() = default;

namespace motion
{
namespace control
{
namespace mpc_controller
{
using motion_common::Command;
using motion_common::Heading;
using motion_common::Index;
using motion_common::Point;
using motion_common::Real;
using motion_common::State;
using motion_common::Trajectory;

using controller_common::BehaviorConfig;
using motion_common::LimitsConfig;
using motion_common::OptimizationConfig;
using motion_common::StateWeight;
using motion_common::VehicleConfig;

enum class Interpolation : uint8_t { YES = 0U, NO = 1U };

struct MPC_CONTROLLER_PUBLIC EnvironmentForces
{
  Real tau_u;
  Real tau_v;
  Real tau_r;
};

/// \brief A configuration class for the MpcController
class MPC_CONTROLLER_PUBLIC Config
{
public:
  Config(
    const EnvironmentForces & env_forces,
    const LimitsConfig & limits,
    const VehicleConfig & vehicle_param,
    const BehaviorConfig & behavior,
    const OptimizationConfig & optimization_param,
    std::chrono::nanoseconds sample_period_tolerance,
    std::chrono::nanoseconds control_lookahead_duration,
    Interpolation interpolation_option);
  MPC_CONTROLLER_COPY_MOVE_ASSIGNABLE(Config)

  const EnvironmentForces & env_forces() const noexcept;
  const LimitsConfig & limits() const noexcept;
  const VehicleConfig & vehicle_param() const noexcept;
  const BehaviorConfig & behavior() const noexcept;
  const OptimizationConfig & optimization_param() const noexcept;
  std::chrono::nanoseconds sample_period_tolerance() const noexcept;
  std::chrono::nanoseconds control_lookahead_duration() const noexcept;
  bool do_interpolate() const noexcept;

private:
  EnvironmentForces m_env_forces;
  LimitsConfig m_limits;
  VehicleConfig m_vehicle_param;
  BehaviorConfig m_behavior_param;
  OptimizationConfig m_optimization_param;
  std::chrono::nanoseconds m_sample_period_tolerance;
  std::chrono::nanoseconds m_control_lookahead_duration;
  bool m_do_interpolate;
};  // class Config

// TODO need change
struct MPC_CONTROLLER_PUBLIC ControlDerivatives
{
  Real jerk_mps3;
  Real steer_angle_rate_rps;
};  // struct ControlDerivatives
}  // namespace mpc_controller
}  // namespace control
}  // namespace motion
#endif  // MPC_CONTROLLER__CONFIG_HPP_
