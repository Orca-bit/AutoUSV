//
// Created by liuhao on 2022/1/11.
//

#include <nmhe_estimator/config.hpp>

namespace motion
{
namespace control
{
namespace nmhe_estimator
{
Weights::Weights(
  Real u_weight,
  Real v_weight,
  Real r_weight,
  Real env_u_weight,
  Real env_v_weight,
  Real env_r_weight,
  Real u_noise_weight,
  Real v_noise_weight,
  Real r_noise_weight,
  Real env_u_noise_weight,
  Real env_v_noise_weight,
  Real env_r_noise_weight)
: m_u_weight(u_weight),
  m_v_weight(v_weight),
  m_r_weight(r_weight),
  m_env_u_weight(env_u_weight),
  m_env_v_weight(env_v_weight),
  m_env_r_weight(env_r_weight),
  m_u_noise_weight(u_noise_weight),
  m_v_noise_weight(v_noise_weight),
  m_r_noise_weight(r_noise_weight),
  m_env_u_noise_weight(env_u_noise_weight),
  m_env_v_noise_weight(env_v_noise_weight),
  m_env_r_noise_weight(env_r_noise_weight)
{
}
Real Weights::u_weight() const noexcept
{
  return m_u_weight;
}
Real Weights::v_weight() const noexcept
{
  return m_v_weight;
}
Real Weights::r_weight() const noexcept
{
  return m_r_weight;
}
Real Weights::env_u_weight() const noexcept
{
  return m_env_u_weight;
}
Real Weights::env_v_weight() const noexcept
{
  return m_env_v_weight;
}
Real Weights::env_r_weight() const noexcept
{
  return m_env_r_weight;
}
Real Weights::u_noise_weight() const noexcept
{
  return m_u_noise_weight;
}
Real Weights::v_noise_weight() const noexcept
{
  return m_v_noise_weight;
}
Real Weights::r_noise_weight() const noexcept
{
  return m_r_noise_weight;
}
Real Weights::env_u_noise_weight() const noexcept
{
  return m_env_u_noise_weight;
}
Real Weights::env_v_noise_weight() const noexcept
{
  return m_env_v_noise_weight;
}
Real Weights::env_r_noise_weight() const noexcept
{
  return m_env_r_noise_weight;
}
/////////////////////////////////////////////////////////////////////////////////

OptimizationConfig::OptimizationConfig(Weights q0, Weights q, Weights r) : m_q0(q0), m_q(q), m_r(r)
{
}
Weights OptimizationConfig::q0() const noexcept
{
  return m_q0;
}
Weights OptimizationConfig::q() const noexcept
{
  return m_q;
}
Weights OptimizationConfig::r() const noexcept
{
  return m_r;
}
/////////////////////////////////////////////////////////////////////////////////

Config::Config(VehicleConfig vehicle_param, OptimizationConfig optimization_param)
: m_vehicle_param(vehicle_param), m_optimization_param(optimization_param)
{
}
const VehicleConfig & Config::vehicle_param() const noexcept
{
  return m_vehicle_param;
}
const OptimizationConfig & Config::optimization_param() const noexcept
{
  return m_optimization_param;
}
}  // namespace nmhe_estimator
}  // namespace control
}  // namespace motion