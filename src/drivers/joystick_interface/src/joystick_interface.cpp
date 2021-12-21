//
// Created by liuhao on 2021/11/8.
//

#include "joystick_interface/joystick_interface.hpp"

#include <utility>

namespace joystick_interface
{
JoystickInterface::JoystickInterface(
  AxisMap axis_map, AxisScaleMap axis_scale_map, AxisScaleMap axis_offset_map)
: m_axis_map(std::move(axis_map)),
  m_axis_scale_map(std::move(axis_scale_map)),
  m_axis_offset_map(std::move(axis_offset_map))
{
}

template <>
JoystickInterface::VehicleControl JoystickInterface::compute_command(
  const sensor_msgs::msg::Joy & msg)
{
  VehicleControl res{};
  res.stamp = msg.header.stamp;
  axis_value(msg, Axes::LEFT_CMD, res.left_cmd);
  axis_value(msg, Axes::RIGHT_CMD, res.right_cmd);
  return res;
}

///TODO: not implemented yet
template <>
JoystickInterface::HighLevelControl JoystickInterface::compute_command(
  const sensor_msgs::msg::Joy &)
{
  return HighLevelControl{};
}

template <typename T>
void JoystickInterface::axis_value(const sensor_msgs::msg::Joy & msg, Axes axis, T & value) const
{
  const auto axis_iter = m_axis_map.find(axis);
  if (m_axis_map.end() == axis_iter) {
    return;
  }
  const auto axis_idx = axis_iter->second;
  if (axis_idx >= msg.axes.size()) {
    return;
  }
  const auto scale_iter = m_axis_scale_map.find(axis);
  const auto scale = scale_iter == m_axis_scale_map.end() ? DEFAULT_SCALE : scale_iter->second;
  const auto val_raw = msg.axes[axis_idx] * scale;
  const auto offset_iter = m_axis_offset_map.find(axis);
  const auto offset = offset_iter == m_axis_offset_map.end() ? DEFAULT_OFFSET : offset_iter->second;
  using ValT = std::decay_t<decltype(value)>;
  value = static_cast<ValT>(val_raw + offset);
}

}  // namespace joystick_interface