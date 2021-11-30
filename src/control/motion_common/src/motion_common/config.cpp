#include "motion_common/config.hpp"

#include <limits>
#include <stdexcept>

namespace motion
{
namespace motion_common
{
LimitsConfig::Extremum::Extremum(Real min, Real max) : m_min{min}, m_max{max}
{
  if (min >= max - std::numeric_limits<decltype(max)>::epsilon()) {
    throw std::domain_error{"Extremum: min >= max - epsilon"};
  }
}

Real LimitsConfig::Extremum::min() const noexcept
{
  return m_min;
}
Real LimitsConfig::Extremum::max() const noexcept
{
  return m_max;
}

////////////////////////////////////////////////////////////////////////////////
LimitsConfig::LimitsConfig(
  Extremum longitudinal_velocity_mps,
  Extremum lateral_velocity_mps,
  Extremum acceleration_mps2,
  Extremum yaw_rate_rps,
  Extremum jerk_mps3,
  Extremum thruster_force_newton)
: m_longitudinal_velocity_limits_mps{longitudinal_velocity_mps},
  m_lateral_velocity_limits_mps{lateral_velocity_mps},
  m_acceleration_limits_mps2{acceleration_mps2},
  m_yaw_rate_limits_rps{yaw_rate_rps},
  m_jerk_limits_mps3{jerk_mps3},
  m_thruster_force_newton{thruster_force_newton}
{
}

LimitsConfig::Extremum LimitsConfig::longitudinal_velocity() const noexcept
{
  return m_longitudinal_velocity_limits_mps;
}
LimitsConfig::Extremum LimitsConfig::lateral_velocity() const noexcept
{
  return m_lateral_velocity_limits_mps;
}
LimitsConfig::Extremum LimitsConfig::acceleration() const noexcept
{
  return m_acceleration_limits_mps2;
}
LimitsConfig::Extremum LimitsConfig::jerk() const noexcept
{
  return m_jerk_limits_mps3;
}
LimitsConfig::Extremum LimitsConfig::yaw_rate() const noexcept
{
  return m_yaw_rate_limits_rps;
}
LimitsConfig::Extremum LimitsConfig::thruster_force() const noexcept
{
  return m_thruster_force_newton;
}

////////////////////////////////////////////////////////////////////////////////
VehicleConfig::VehicleConfig(
  Real length_cg_front_axel_m,
  Real length_cg_rear_axel_m,
  Real length_cg_thrusters_long_m,
  Real length_cg_thrusters_lateral_m,
  Real front_cornering_stiffness_N,
  Real rear_cornering_stiffness_N,
  Real mass11_kg,
  Real mass22_kg,
  Real mass33_kgm2,
  Real damping11_kgm_s,
  Real damping22_kgm_s,
  Real damping33_kgm2_s,
  Real width_m)
: m_length_cg_to_front_axel_m{length_cg_front_axel_m},
  m_length_cg_to_rear_axel_m{length_cg_rear_axel_m},
  m_length_cg_thrusters_long_m{length_cg_thrusters_long_m},
  m_length_cg_thrusters_lateral_m{length_cg_thrusters_lateral_m},
  m_front_cornering_stiffness_N{front_cornering_stiffness_N},
  m_rear_cornering_stiffness_N{rear_cornering_stiffness_N},
  m_mass11_kg{mass11_kg},
  m_mass22_kg{mass22_kg},
  m_mass33_kgm2{mass33_kgm2},
  m_damping11_kgm_s{damping11_kgm_s},
  m_damping22_kgm_s{damping22_kgm_s},
  m_damping33_kgm2_s{damping33_kgm2_s},
  m_width_m{width_m}
{
}

Real VehicleConfig::length_cg_front_axel() const noexcept
{
  return m_length_cg_to_front_axel_m;
}
Real VehicleConfig::length_cg_rear_axel() const noexcept
{
  return m_length_cg_to_rear_axel_m;
}
Real VehicleConfig::length_cg_thrusters_long_m() const noexcept
{
  return m_length_cg_thrusters_long_m;
}
Real VehicleConfig::length_cg_thrusters_lateral_m() const noexcept
{
  return m_length_cg_thrusters_lateral_m;
}
Real VehicleConfig::front_cornering_stiffness() const noexcept
{
  return m_front_cornering_stiffness_N;
}
Real VehicleConfig::rear_cornering_stiffness() const noexcept
{
  return m_rear_cornering_stiffness_N;
}
Real VehicleConfig::mass11() const noexcept
{
  return m_mass11_kg;
}
Real VehicleConfig::mass22() const noexcept
{
  return m_mass22_kg;
}
Real VehicleConfig::mass33() const noexcept
{
  return m_mass33_kgm2;
}
Real VehicleConfig::damping_11() const noexcept
{
  return m_damping11_kgm_s;
}
Real VehicleConfig::damping_22() const noexcept
{
  return m_damping22_kgm_s;
}
Real VehicleConfig::damping_33() const noexcept
{
  return m_damping33_kgm2_s;
}
Real VehicleConfig::width() const noexcept
{
  return m_width_m;
}

////////////////////////////////////////////////////////////////////////////////
StateWeight::StateWeight(
  Real pose_x,
  Real pose_y,
  Real heading,
  Real longitudinal_velocity,
  Real lateral_velocity,
  Real yaw_rate,
  Real acceleration,
  Real jerk,
  Real left_cmd,
  Real right_cmd)
: m_pose_x_weight{pose_x},
  m_pose_y_weight{pose_y},
  m_heading_weight{heading},
  m_longitudinal_velocity_weight{longitudinal_velocity},
  m_lateral_velocity_weight{lateral_velocity},
  m_yaw_rate_weight{yaw_rate},
  m_acceleration_weight{acceleration},
  m_jerk_weight{jerk},
  m_left_cmd_weight{left_cmd},
  m_right_cmd_weight{right_cmd}
{
  if (pose_x < Real{}) {  // zero initialization
    throw std::domain_error{"Pose x weight is negative!"};
  }
  if (pose_y < Real{}) {  // zero initialization
    throw std::domain_error{"Pose y weight is negative!"};
  }
  if (heading < Real{}) {  // zero initialization
    throw std::domain_error{"Heading weight is negative!"};
  }
  if (longitudinal_velocity < Real{}) {  // zero initialization
    throw std::domain_error{"Longitudinal weight is negative!"};
  }
  if (lateral_velocity < Real{}) {  // zero initialization
    throw std::domain_error{"Lateral velocity weight is negative!"};
  }
  if (yaw_rate < Real{}) {  // zero initialization
    throw std::domain_error{"Yaw rate weight is negative!"};
  }
  if (acceleration < Real{}) {  // zero initialization
    throw std::domain_error{"Acceleration weight is negative!"};
  }
  if (jerk < Real{}) {  // zero initialization
    throw std::domain_error{"Jerk weight is negative!"};
  }
  if (left_cmd < Real{}) {
    throw std::domain_error{"Left cmd weight is negative!"};
  }
  if (right_cmd < Real{}) {
    throw std::domain_error{"Right cmd weight is negative!"};
  }
}

Real StateWeight::pose_x() const noexcept
{
  return m_pose_x_weight;
}

Real StateWeight::pose_y() const noexcept
{
  return m_pose_y_weight;
}

Real StateWeight::heading() const noexcept
{
  return m_heading_weight;
}

Real StateWeight::longitudinal_velocity() const noexcept
{
  return m_longitudinal_velocity_weight;
}

Real StateWeight::lateral_velocity() const noexcept
{
  return m_lateral_velocity_weight;
}

Real StateWeight::yaw_rate() const noexcept
{
  return m_yaw_rate_weight;
}

Real StateWeight::acceleration() const noexcept
{
  return m_acceleration_weight;
}

Real StateWeight::jerk() const noexcept
{
  return m_jerk_weight;
}

Real StateWeight::left_cmd() const noexcept
{
  return m_left_cmd_weight;
}

Real StateWeight::right_cmd() const noexcept
{
  return m_right_cmd_weight;
}

////////////////////////////////////////////////////////////////////////////////
OptimizationConfig::OptimizationConfig(StateWeight nominal_weights, StateWeight terminal_weights)
: m_nominal_weights{nominal_weights}, m_terminal_weights{terminal_weights}
{
}

StateWeight OptimizationConfig::nominal() const noexcept
{
  return m_nominal_weights;
}

StateWeight OptimizationConfig::terminal() const noexcept
{
  return m_terminal_weights;
}
}  // namespace motion_common
}  // namespace motion
