#ifndef MOTION_COMMON__CONFIG_HPP_
#define MOTION_COMMON__CONFIG_HPP_

#include <motion_common/motion_common.hpp>
#include <motion_common/visibility_control.hpp>

#define MOTION_COMMON_COPY_MOVE_ASSIGNABLE(Class) \
  Class(const Class &) = default;                 \
  Class(Class &&) = default;                      \
  Class & operator=(const Class &) = default;     \
  Class & operator=(Class &&) = default;          \
  ~Class() = default;

namespace motion
{
namespace motion_common
{
using motion_common::Command;
using motion_common::Heading;
using motion_common::Index;
using motion_common::Point;
using motion_common::Real;
using motion_common::State;
using motion_common::Trajectory;

/// Extreme values for state/control variables
class MOTION_COMMON_PUBLIC LimitsConfig
{
public:
  /// \brief Class representing min and max values for a variable
  class Extremum
  {
  public:
    Extremum(Real min, Real max);
    MOTION_COMMON_COPY_MOVE_ASSIGNABLE(Extremum)

    Real min() const noexcept;
    Real max() const noexcept;

  private:
    Real m_min;
    Real m_max;
  };  // class Extremum

  LimitsConfig(
    Extremum longitudinal_velocity_mps,
    Extremum lateral_velocity_mps,
    Extremum acceleration_mps2,
    Extremum yaw_rate_rps,
    Extremum jerk_mps3,
    Extremum thruster_force_newton);
  MOTION_COMMON_COPY_MOVE_ASSIGNABLE(LimitsConfig)

  Extremum longitudinal_velocity() const noexcept;
  Extremum lateral_velocity() const noexcept;
  Extremum acceleration() const noexcept;
  Extremum jerk() const noexcept;
  Extremum yaw_rate() const noexcept;
  Extremum thruster_force() const noexcept;

private:
  Extremum m_longitudinal_velocity_limits_mps;
  Extremum m_lateral_velocity_limits_mps;
  Extremum m_acceleration_limits_mps2;
  Extremum m_yaw_rate_limits_rps;
  Extremum m_jerk_limits_mps3;
  Extremum m_thruster_force_newton;
};  // class LimitsConfig

/// Vehicle parameters specifying vehicle's handling performance
class MOTION_COMMON_PUBLIC VehicleConfig
{
public:
  VehicleConfig(
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
    Real width_m);
  MOTION_COMMON_COPY_MOVE_ASSIGNABLE(VehicleConfig)

  Real length_cg_front_axel() const noexcept;
  Real length_cg_rear_axel() const noexcept;
  Real length_cg_thrusters_long_m() const noexcept;
  Real length_cg_thrusters_lateral_m() const noexcept;
  Real front_cornering_stiffness() const noexcept;
  Real rear_cornering_stiffness() const noexcept;
  Real mass11() const noexcept;
  Real mass22() const noexcept;
  Real mass33() const noexcept;
  Real damping_11() const noexcept;
  Real damping_22() const noexcept;
  Real damping_33() const noexcept;
  Real width() const noexcept;

private:
  Real m_length_cg_to_front_axel_m;
  Real m_length_cg_to_rear_axel_m;
  Real m_length_cg_thrusters_long_m;
  Real m_length_cg_thrusters_lateral_m;
  Real m_front_cornering_stiffness_N;
  Real m_rear_cornering_stiffness_N;
  Real m_mass11_kg;
  Real m_mass22_kg;
  Real m_mass33_kgm2;
  Real m_damping11_kgm_s;
  Real m_damping22_kgm_s;
  Real m_damping33_kgm2_s;
  Real m_width_m;
};  // class VehicleConfig

/// \brief Specifies the weights used for particular state weights in the least-squares objective
///        function of the mpc solver
class MOTION_COMMON_PUBLIC StateWeight
{
public:
  StateWeight(
    Real pose_x,
    Real pose_y,
    Real heading,
    Real longitudinal_velocity,
    Real lateral_velocity,
    Real yaw_rate,
    Real acceleration,
    Real jerk);
  MOTION_COMMON_COPY_MOVE_ASSIGNABLE(StateWeight)

  Real pose_x() const noexcept;
  Real pose_y() const noexcept;
  Real heading() const noexcept;
  Real longitudinal_velocity() const noexcept;
  Real lateral_velocity() const noexcept;
  Real yaw_rate() const noexcept;
  Real acceleration() const noexcept;
  Real jerk() const noexcept;

private:
  Real m_pose_x_weight;
  Real m_pose_y_weight;
  Real m_heading_weight;
  Real m_longitudinal_velocity_weight;
  Real m_lateral_velocity_weight;
  Real m_yaw_rate_weight;
  Real m_acceleration_weight;
  Real m_jerk_weight;
};  // class StateWeight

/// \brief Specifies various parameters specific to the optimization problem and it's behaviors
///        Depending on problem setup, some weights may be ignored
class MOTION_COMMON_PUBLIC OptimizationConfig
{
public:
  OptimizationConfig(StateWeight nominal_weights, StateWeight terminal_weights);
  MOTION_COMMON_COPY_MOVE_ASSIGNABLE(OptimizationConfig)

  StateWeight nominal() const noexcept;
  StateWeight terminal() const noexcept;

private:
  StateWeight m_nominal_weights;
  StateWeight m_terminal_weights;
};  // class OptimizationConfig
}  // namespace motion_common
}  // namespace motion
#endif  // MOTION_COMMON__CONFIG_HPP_
