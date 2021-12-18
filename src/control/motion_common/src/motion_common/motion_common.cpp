#include "motion_common/motion_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "helper_functions/angle_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace motion
{
namespace motion_common
{
////////////////////////////////////////////////////////////////////////////////
bool is_past_point(const Point & state, const Point & pt) noexcept
{
  const auto w = pt.pose.orientation.w;
  const auto z = pt.pose.orientation.z;
  // double angle rule
  const auto c = (w + z) * (w - z);
  const auto s = 2.0 * w * z;

  return is_past_point(state, pt, c, s);
}

////////////////////////////////////////////////////////////////////////////////
bool is_past_point(const Point & state, const Point & current_pt, const Point & next_pt) noexcept
{
  const auto nx = next_pt.pose.position.x - current_pt.pose.position.x;
  const auto ny = next_pt.pose.position.y - current_pt.pose.position.y;

  return is_past_point(state, next_pt, nx, ny);
}

////////////////////////////////////////////////////////////////////////////////
bool is_past_point(const Point & state, const Point & pt, const Double nx, const Double ny) noexcept
{
  const auto dx = (state.pose.position.x - pt.pose.position.x);
  const auto dy = (state.pose.position.y - pt.pose.position.y);

  // Check if state is past last_pt when projected onto the ray defined by heading
  return ((nx * dx) + (ny * dy)) >= -std::numeric_limits<Double>::epsilon();
}

////////////////////////////////////////////////////////////////////////////////
bool heading_ok(const Trajectory & traj)
{
  const auto bad_heading = [](const auto & pt) -> bool {
    const auto real2 = static_cast<Real>(pt.pose.orientation.w * pt.pose.orientation.w);
    const auto imag2 = static_cast<Real>(pt.pose.orientation.z * pt.pose.orientation.z);
    constexpr auto TOL = 1.0E-3F;
    return std::fabs(1.0F - (real2 + imag2)) > TOL;
  };
  const auto bad_it = std::find_if(traj.points.begin(), traj.points.end(), bad_heading);
  // True if there is no bad point
  return bad_it == traj.points.end();
}

////////////////////////////////////////////////////////////////////////////////
void doTransform(
  const Point & t_in,
  Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept
{
  geometry_msgs::msg::PoseStamped p_in;
  p_in.pose = t_in.pose;
  p_in.header.stamp = transform.header.stamp;
  geometry_msgs::msg::PoseStamped p_out;
  tf2::doTransform(p_in, p_out, transform);
  t_out.pose = p_out.pose;
}

////////////////////////////////////////////////////////////////////////////////
void doTransform(
  const State & t_in,
  State & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept
{
  doTransform(t_in.state, t_out.state, transform);
  t_out.header.frame_id = transform.header.frame_id;
}

Double to_angle(Orientation orientation) noexcept
{
  return tf2::getYaw(orientation);
}

////////////////////////////////////////////////////////////////////////////////
Orientation slerp(const Orientation & a, const Orientation & b, const Real t)
{
  const auto t_ = clamp(t, 0.0F, 1.0F);
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a.slerp(quat_b, static_cast<Double>(t_)));
}

////////////////////////////////////////////////////////////////////////////////
Point interpolate(Point a, Point b, Real t)
{
  return interpolate(a, b, t, slerp);
}

////////////////////////////////////////////////////////////////////////////////
void sample(const Trajectory & in, Trajectory & out, std::chrono::nanoseconds period)
{
  sample(in, out, period, slerp);
}

////////////////////////////////////////////////////////////////////////////////
void error(const Point & state, const Point & ref, Diagnostic & out) noexcept
{
  {
    // compute heading normal of reference point
    const auto & q = ref.pose.orientation;
    const auto nx = (q.w * q.w) - (q.z * q.z);
    const auto ny = decltype(nx){2.0} * q.w * q.z;
    // project state onto reference basis
    const auto dx = state.pose.position.x - ref.pose.position.x;
    const auto dy = state.pose.position.y - ref.pose.position.y;
    // normals rotated +90 deg
    out.lateral_error_m = static_cast<Real>((dx * (-ny)) + (dy * nx));
    out.longitudinal_error_m = static_cast<Real>((dx * nx) + (dy * ny));
  }
  out.velocity_error_mps = state.longitudinal_velocity_mps - ref.longitudinal_velocity_mps;
  out.acceleration_error_mps2 = state.acceleration_mps2 - ref.acceleration_mps2;

  using usv::common::helper_functions::wrap_angle;
  out.yaw_error_rad = static_cast<decltype(out.yaw_error_rad)>(
    wrap_angle(to_angle(state.pose.orientation) - to_angle(ref.pose.orientation)));
  out.yaw_rate_error_rps = state.heading_rate_rps - ref.heading_rate_rps;
}
}  // namespace motion_common
}  // namespace motion


namespace geometry_msgs
{
namespace msg
{
Quaternion operator+(Quaternion a, Quaternion b) noexcept
{
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a + quat_b);
}
Quaternion operator-(Quaternion a) noexcept
{
  tf2::Quaternion quat_a;
  tf2::fromMsg(a, quat_a);
  return tf2::toMsg(quat_a * -1.0);
}
Quaternion operator-(Quaternion a, Quaternion b) noexcept
{
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a * quat_b.inverse());
}
}  // namespace msg
}  // namespace geometry_msgs
