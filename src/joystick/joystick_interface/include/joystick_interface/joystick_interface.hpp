//
// Created by liuhao on 2021/11/8.
//

#ifndef JOYSTICK_INTERFACE_JOYSTICK_INTERFACE_HPP
#define JOYSTICK_INTERFACE_JOYSTICK_INTERFACE_HPP

#include <map>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <usv_msgs/msg/raw_control_command.hpp>
#include <usv_msgs/msg/vehicle_state_command.hpp>

namespace joystick_interface {

enum class Axes {
  LEFT_CMD,      ///< For raw control
  RIGHT_CMD,     ///< For raw control
};

enum class Buttons {
  RECORDREPLAY_START_RECORD,
  RECORDREPLAY_START_REPLAY,
  RECORDREPLAY_STOP
};

enum class Recordreplay : uint8_t {
  NOOP = 0u,
  START_RECORD,
  START_REPLAY,
  STOP
};

using AxisValue = decltype(sensor_msgs::msg::Joy::axes)::value_type;
using AxisMap =
    std::map<Axes, decltype(sensor_msgs::msg::Joy::axes)::size_type>;
using AxisScaleMap = std::map<Axes, AxisValue>;
using ButtonMap =
    std::map<Buttons, decltype(sensor_msgs::msg::Joy::buttons)::size_type>;

static constexpr AxisValue DEFAULT_SCALE = 100.0F;
static constexpr AxisValue DEFAULT_OFFSET = 0.0F;
static constexpr AxisValue VELOCITY_INCREMENT = 1.0F;

using usv_msgs::msg::VehicleStateCommand;

class JoystickInterface {
 public:
  JoystickInterface(AxisMap axis_map, AxisScaleMap axis_scale_map,
                    AxisScaleMap axis_offset_map, ButtonMap button_map);
  /// Compute state command
  bool update_state_command(const sensor_msgs::msg::Joy& msg);
  /// Compute control command
  template <typename T>
  T compute_command(const sensor_msgs::msg::Joy& msg);
  void reset_recordplay();
  const VehicleStateCommand& get_state_command();
  const std_msgs::msg::UInt8& get_recordreplay_command();

 private:
  /// Given an active button, update the state command
  bool handle_active_button(Buttons button);
  /// Convert raw axis value with affine transform for type
  template <typename T>
  void axis_value(const sensor_msgs::msg::Joy& msg, Axes axis, T& value) const;

  using RawControl = usv_msgs::msg::RawControlCommand;

  AxisMap m_axis_map{};
  AxisScaleMap m_axis_scale_map{};
  AxisScaleMap m_axis_offset_map{};
  ButtonMap m_button_map{};

  VehicleStateCommand m_state_command{};
  std_msgs::msg::UInt8 m_recordreplay_command{};
};

}  // namespace joystick_interface

#endif  // JOYSTICK_INTERFACE_JOYSTICK_INTERFACE_HPP
