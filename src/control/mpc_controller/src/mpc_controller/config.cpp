#include "mpc_controller/config.hpp"

#include <limits>
#include <stdexcept>

namespace motion
{
namespace control
{
namespace mpc_controller
{
Config::Config(
  const EnvironmentForces & env_forces,
  const LimitsConfig & limits,
  const VehicleConfig & vehicle_param,
  const BehaviorConfig & behavior,
  const OptimizationConfig & optimization_param,
  const std::chrono::nanoseconds sample_period_tolerance,
  const std::chrono::nanoseconds control_lookahead_duration,
  const Interpolation interpolation_option)
: m_env_forces{env_forces},
  m_limits{limits},
  m_vehicle_param{vehicle_param},
  m_behavior_param{behavior},
  m_optimization_param{optimization_param},
  m_sample_period_tolerance{sample_period_tolerance},
  m_control_lookahead_duration{control_lookahead_duration},
  m_do_interpolate{interpolation_option == Interpolation::YES}
{
  if (sample_period_tolerance < decltype(sample_period_tolerance)::zero()) {
    throw std::domain_error{"Sample period tolerance must be positive"};
  }
  // Does it actually _have_ to be positive?
  if (control_lookahead_duration < decltype(control_lookahead_duration)::zero()) {
    throw std::domain_error{"Control lookahead duration must be positive"};
  }
}

const EnvironmentForces & Config::env_forces() const noexcept
{
  return m_env_forces;
}
const LimitsConfig & Config::limits() const noexcept
{
  return m_limits;
}
const VehicleConfig & Config::vehicle_param() const noexcept
{
  return m_vehicle_param;
}
const BehaviorConfig & Config::behavior() const noexcept
{
  return m_behavior_param;
}
const OptimizationConfig & Config::optimization_param() const noexcept
{
  return m_optimization_param;
}
std::chrono::nanoseconds Config::sample_period_tolerance() const noexcept
{
  return m_sample_period_tolerance;
}
std::chrono::nanoseconds Config::control_lookahead_duration() const noexcept
{
  return m_control_lookahead_duration;
}
bool Config::do_interpolate() const noexcept
{
  return m_do_interpolate;
}
}  // namespace mpc_controller
}  // namespace control
}  // namespace motion
