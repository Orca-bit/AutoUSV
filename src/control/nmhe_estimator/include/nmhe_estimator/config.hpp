//
// Created by liuhao on 2022/1/11.
//

#ifndef NMHE_ESTIMATOR_CONFIG_HPP
#define NMHE_ESTIMATOR_CONFIG_HPP

#include <motion_common/config.hpp>
#include <nmhe_estimator/visibility_control.hpp>

#define NMHE_ESTIMATOR_COPY_MOVE_ASSIGNABLE(Class) \
  Class(const Class &) = default;                  \
  Class(Class &&) = default;                       \
  Class & operator=(const Class &) = default;      \
  Class & operator=(Class &&) = default;           \
  ~Class() = default;

namespace motion
{
namespace control
{
namespace nmhe_estimator
{
using motion_common::Index;
using motion_common::Real;
using motion_common::VehicleConfig;

class NMHE_ESTIMATOR_PUBLIC Weights
{
public:
  Weights(
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
    Real env_r_noise_weight);
  NMHE_ESTIMATOR_COPY_MOVE_ASSIGNABLE(Weights)

  Real u_weight() const noexcept;
  Real v_weight() const noexcept;
  Real r_weight() const noexcept;
  Real env_u_weight() const noexcept;
  Real env_v_weight() const noexcept;
  Real env_r_weight() const noexcept;
  Real u_noise_weight() const noexcept;
  Real v_noise_weight() const noexcept;
  Real r_noise_weight() const noexcept;
  Real env_u_noise_weight() const noexcept;
  Real env_v_noise_weight() const noexcept;
  Real env_r_noise_weight() const noexcept;

private:
  Real m_u_weight;
  Real m_v_weight;
  Real m_r_weight;
  Real m_env_u_weight;
  Real m_env_v_weight;
  Real m_env_r_weight;
  Real m_u_noise_weight;
  Real m_v_noise_weight;
  Real m_r_noise_weight;
  Real m_env_u_noise_weight;
  Real m_env_v_noise_weight;
  Real m_env_r_noise_weight;
};

class NMHE_ESTIMATOR_PUBLIC OptimizationConfig
{
public:
  OptimizationConfig(Weights q0, Weights q, Weights r);
  NMHE_ESTIMATOR_COPY_MOVE_ASSIGNABLE(OptimizationConfig)

  Weights q0() const noexcept;
  Weights q() const noexcept;
  Weights r() const noexcept;

private:
  Weights m_q0;
  Weights m_q;
  Weights m_r;
};

class NMHE_ESTIMATOR_PUBLIC Config
{
public:
  Config(VehicleConfig vehicle_param, OptimizationConfig optimization_param);
  NMHE_ESTIMATOR_COPY_MOVE_ASSIGNABLE(Config)

  const VehicleConfig & vehicle_param() const noexcept;
  const OptimizationConfig & optimization_param() const noexcept;

private:
  VehicleConfig m_vehicle_param;
  OptimizationConfig m_optimization_param;
};

}  // namespace nmhe_estimator
}  // namespace control
}  // namespace motion

#endif  // NMHE_ESTIMATOR_CONFIG_HPP
