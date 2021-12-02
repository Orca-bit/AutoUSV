//
// Created by liuhao on 2021/11/8.
//

#ifndef JOYSTICK_INTERFACE_JOYSTICK_INTERFACE_HPP
#define JOYSTICK_INTERFACE_JOYSTICK_INTERFACE_HPP

#include <map>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <usv_msgs/msg/vehicle_control_command.hpp>

namespace joystick_interface
{

enum class Axes {
  LEFT_CMD,   ///< For raw control
  RIGHT_CMD,  ///< For raw control
};

using AxisValue = decltype(sensor_msgs::msg::Joy::axes)::value_type;
using AxisMap = std::map<Axes, decltype(sensor_msgs::msg::Joy::axes)::size_type>;
using AxisScaleMap = std::map<Axes, AxisValue>;

static constexpr AxisValue DEFAULT_SCALE = 100.0F;
static constexpr AxisValue DEFAULT_OFFSET = 0.0F;

class JoystickInterface
{
public:
  JoystickInterface(AxisMap axis_map, AxisScaleMap axis_scale_map, AxisScaleMap axis_offset_map);
  /// Compute control command
  template <typename T> T compute_command(const sensor_msgs::msg::Joy & msg);

private:
  /// Convert raw axis value with affine transform for type
  template <typename T>
  void axis_value(const sensor_msgs::msg::Joy & msg, Axes axis, T & value) const;

  using VehicleControl = usv_msgs::msg::VehicleControlCommand;

  AxisMap m_axis_map{};
  AxisScaleMap m_axis_scale_map{};
  AxisScaleMap m_axis_offset_map{};
};

}  // namespace joystick_interface

#endif  // JOYSTICK_INTERFACE_JOYSTICK_INTERFACE_HPP
