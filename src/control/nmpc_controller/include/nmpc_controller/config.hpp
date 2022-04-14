#ifndef NMPC_CONTROLLER__CONFIG_HPP_
#define NMPC_CONTROLLER__CONFIG_HPP_

#include <controller_common/controller_base.hpp>
#include <motion_common/config.hpp>
#include <nmpc_controller/visibility_control.hpp>

#define NMPC_CONTROLLER_COPY_MOVE_ASSIGNABLE(Class) \
  Class(const Class &) = default;                  \
  Class(Class &&) = default;                       \
  Class & operator=(const Class &) = default;      \
  Class & operator=(Class &&) = default;           \
  ~Class() = default;

namespace motion
{
namespace control
{
namespace nmpc_controller
{
using motion_common::Command;
using motion_common::Index;
using motion_common::Point;
using motion_common::Real;
using motion_common::State;
using motion_common::Trajectory;

using controller_common::BehaviorConfig;
using motion_common::EnvironmentForces;
using motion_common::LimitsConfig;
using motion_common::OptimizationConfig;
using motion_common::StateWeight;
using motion_common::VehicleConfig;

enum class Interpolation : uint8_t { YES = 0U, NO = 1U };

/// \brief A configuration class for the NmpcController
class NMPC_CONTROLLER_PUBLIC Config
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
  NMPC_CONTROLLER_COPY_MOVE_ASSIGNABLE(Config)

  const EnvironmentForces & env_forces() const noexcept;
  const LimitsConfig & limits() const noexcept;
  const VehicleConfig & vehicle_param() const noexcept;
  const BehaviorConfig & behavior() const noexcept;
  const OptimizationConfig & optimization_param() const noexcept;
  std::chrono::nanoseconds sample_period_tolerance() const noexcept;
  std::chrono::nanoseconds control_lookahead_duration() const noexcept;
  bool do_interpolate() const noexcept;
  void set_env_forces(const EnvironmentForces & env_forces);

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

}  // namespace mpc_controller
}  // namespace control
}  // namespace motion
#endif  // NMPC_CONTROLLER__CONFIG_HPP_
