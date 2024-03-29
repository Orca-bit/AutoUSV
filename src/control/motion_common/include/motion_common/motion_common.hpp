#ifndef MOTION_COMMON__MOTION_COMMON_HPP_
#define MOTION_COMMON__MOTION_COMMON_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>

#include <motion_common/visibility_control.hpp>
#include <time_utils/time_utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <usv_msgs/msg/control_diagnostic.hpp>
#include <usv_msgs/msg/env_estimation.hpp>
#include <usv_msgs/msg/trajectory.hpp>
#include <usv_msgs/msg/vehicle_control_command.hpp>
#include <usv_msgs/msg/vehicle_kinematic_state.hpp>

namespace motion
{
namespace motion_common
{
// Use same representation as message type
using Real = decltype(usv_msgs::msg::TrajectoryPoint::longitudinal_velocity_mps);
using Command = usv_msgs::msg::VehicleControlCommand;
using Diagnostic = usv_msgs::msg::ControlDiagnostic;
using State = usv_msgs::msg::VehicleKinematicState;
using Trajectory = usv_msgs::msg::Trajectory;
using Orientation = geometry_msgs::msg::Quaternion;
using Double = decltype(Orientation::x);
using Index = decltype(Trajectory::points)::size_type;
using Point = decltype(Trajectory::points)::value_type;
using EnvironmentForces = usv_msgs::msg::EnvEstimation;

/// Check if a state is past a given trajectory point, assuming heading is correct
MOTION_COMMON_PUBLIC bool is_past_point(const Point & state, const Point & pt) noexcept;
/// Check if a state is past a given trajectory point given the last (aka current)
/// trajectory point
MOTION_COMMON_PUBLIC bool is_past_point(
  const Point & state, const Point & current_pt, const Point & next_pt) noexcept;
/// Given a normal, determine if state is past a point
MOTION_COMMON_PUBLIC
bool is_past_point(const Point & state, const Point & pt, Double nx, Double ny) noexcept;

/// Advance to the first trajectory point past state according to criterion is_past_point
template <typename IsPastPointF>
Index update_reference_index(
  const Trajectory & traj, const State & state, Index start_idx, IsPastPointF is_past_point)
{
  // Invariant: m_reference_trajectory.points.size > 0U
  if (traj.points.empty()) {
    return 0U;
  }
  auto next_idx = start_idx;
  for (auto idx = start_idx; idx < traj.points.size() - 1U; ++idx) {
    const auto current_pt = traj.points[idx];
    const auto next_pt = traj.points[idx + 1U];

    // If I'm not past the next point, then I'm done
    if (!is_past_point(state, current_pt, next_pt, traj)) {
      break;
    }
    // Otherwise, update
    next_idx = idx + 1U;
  }
  return next_idx;
}

/// Check that all heading values in a trajectory are normalized 2D quaternions
MOTION_COMMON_PUBLIC bool heading_ok(const Trajectory & traj);

////////////////////////////////////////////////////////////////////////////////
// Operations relating to algebraic manipulation of VehicleKinematicState and TrajectoryPoint

/// Apply transform to TrajectoryPoint
MOTION_COMMON_PUBLIC void doTransform(
  const Point & t_in,
  Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept;
/// Apply transform to VehicleKinematicState
MOTION_COMMON_PUBLIC void doTransform(
  const State & t_in,
  State & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept;

/// Converts 3D quaternion to simple heading representation
MOTION_COMMON_PUBLIC Double to_angle(Orientation orientation) noexcept;

/// \brief Converts angles into a corresponding Orientation
/// \tparam RealT a floating point type
/// \param[in] roll angle to use as roll of the Orientation [radians]
/// \param[in] pitch angle to use as pitch of the Orientation [radians]
/// \param[in] yaw angle to use as yaw of the Orientation [radians]
/// \returns A converted Orientation object
template <typename RealT> Orientation from_angles(RealT roll, RealT pitch, RealT yaw) noexcept
{
  static_assert(std::is_floating_point<RealT>::value, "angle must be floating point");
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quat);
}

/// \brief Converts a heading angle into a corresponding Orientation
/// \tparam RealT a floating point type
/// \param[in] angle heading angle to use as yaw of the Orientation [radians]
/// \returns A converted Orientation object
template<typename RealT>
Orientation from_angle(RealT angle) noexcept
{
  return from_angles(RealT{}, RealT{}, angle);
}

/// Standard clamp implementation
template <typename T> T clamp(T val, T min, T max)
{
  if (max < min) {
    throw std::domain_error{"max < min"};
  }
  return std::min(max, std::max(min, val));
}

/// Standard linear interpolation
template <typename T, typename RealT = double> T interpolate(T a, T b, RealT t)
{
  static_assert(std::is_floating_point<RealT>::value, "t must be floating point");
  t = clamp(t, RealT{0.0}, RealT{1.0});
  const auto del = b - a;
  return static_cast<T>(t * static_cast<RealT>(del)) + a;
}

/// Spherical linear interpolation
MOTION_COMMON_PUBLIC Orientation slerp(const Orientation & a, const Orientation & b, Real t);

/// Trajectory point interpolation
template<typename SlerpF>
Point interpolate(Point a, Point b, Real t, SlerpF slerp_fn)
{
  Point ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  {
    const auto dt0 = time_utils::from_message(a.time_from_start);
    const auto dt1 = time_utils::from_message(b.time_from_start);
    ret.time_from_start = time_utils::to_message(time_utils::interpolate(dt0, dt1, t));
  }
  ret.pose.position.x = interpolate(a.pose.position.x, b.pose.position.x, t);
  ret.pose.position.y = interpolate(a.pose.position.y, b.pose.position.y, t);
  ret.pose.orientation = slerp_fn(a.pose.orientation, b.pose.orientation, t);
  ret.longitudinal_velocity_mps =
    interpolate(a.longitudinal_velocity_mps, b.longitudinal_velocity_mps, t);
  ret.lateral_velocity_mps = interpolate(a.lateral_velocity_mps, b.lateral_velocity_mps, t);
  ret.acceleration_mps2 = interpolate(a.acceleration_mps2, b.acceleration_mps2, t);
  ret.heading_rate_rps = interpolate(a.heading_rate_rps, b.heading_rate_rps, t);

  return ret;
}

/// Default point interpolation, currently uses nlerp
MOTION_COMMON_PUBLIC Point interpolate(Point a, Point b, Real t);

/// Sample a trajectory using interpolation; does not extrapolate
template <typename SlerpF>
void sample(
  const Trajectory & in, Trajectory & out, std::chrono::nanoseconds period, SlerpF slerp_fn)
{
  out.points.clear();
  out.header = in.header;
  if (in.points.empty()) {
    return;
  }
  // Determine number of points
  using time_utils::from_message;
  const auto last_time = from_message(in.points.back().time_from_start);
  auto count_ = last_time / period;
  // should round down
  using SizeT = typename decltype(in.points)::size_type;
  const auto count =
    static_cast<SizeT>(std::min(count_, static_cast<decltype(count_)>(in.CAPACITY)));
  out.points.reserve(count);
  // Insert first point
  out.points.push_back(in.points.front());
  // Add remaining points
  auto ref_duration = period;
  auto after_current_ref_idx = SizeT{1};
  for (auto idx = SizeT{1}; idx < count; ++idx) {
    // Determine next reference index
    for (auto jdx = after_current_ref_idx; jdx < in.points.size(); ++jdx) {
      const auto & pt = in.points[jdx];
      if (from_message(pt.time_from_start) > ref_duration) {
        after_current_ref_idx = jdx;
        break;
      }
    }
    // Interpolate
    {
      const auto & curr_pt = in.points[after_current_ref_idx - 1U];
      const auto & next_pt = in.points[after_current_ref_idx];
      const auto dt = std::chrono::duration_cast<std::chrono::duration<Real>>(
        from_message(next_pt.time_from_start) - from_message(curr_pt.time_from_start));
      const auto dt_ = std::chrono::duration_cast<std::chrono::duration<Real>>(
        ref_duration - from_message(curr_pt.time_from_start));
      const auto t = dt_.count() / dt.count();
      out.points.push_back(interpolate(curr_pt, next_pt, t, slerp_fn));
    }
    ref_duration += period;
  }
}

/// Trajectory sampling with default interpolation
MOTION_COMMON_PUBLIC void sample(
  const Trajectory & in, Trajectory & out, std::chrono::nanoseconds period);

/// Diagnostic header stuff
MOTION_COMMON_PUBLIC
void error(const Point & state, const Point & ref, Diagnostic & out) noexcept;
}  // namespace motion_common
}  // namespace motion

namespace geometry_msgs
{
namespace msg
{
/// Addition operator
MOTION_COMMON_PUBLIC Quaternion operator+(Quaternion a, Quaternion b) noexcept;
/// Unary minus
MOTION_COMMON_PUBLIC Quaternion operator-(Quaternion a) noexcept;
/// Difference operator
MOTION_COMMON_PUBLIC Quaternion operator-(Quaternion a, Quaternion b) noexcept;
}  // namespace msg
}  // namespace geometry_msgs
#endif  // MOTION_COMMON__MOTION_COMMON_HPP_
