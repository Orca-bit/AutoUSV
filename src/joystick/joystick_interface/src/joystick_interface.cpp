//
// Created by liuhao on 2021/11/8.
//

#include "../include/joystick_interface/joystick_interface.hpp"

#include <utility>

namespace joystick_interface {
JoystickInterface::JoystickInterface(AxisMap axis_map,
                                     AxisScaleMap axis_scale_map,
                                     AxisScaleMap axis_offset_map,
                                     ButtonMap button_map)
    : m_axis_map(std::move(axis_map)),
      m_axis_scale_map(std::move(axis_scale_map)),
      m_axis_offset_map(std::move(axis_offset_map)),
      m_button_map(std::move(button_map)) {}

bool JoystickInterface::update_state_command(const sensor_msgs::msg::Joy &msg) {
  bool res = false;
  m_state_command = decltype(m_state_command){};
  m_state_command.stamp = msg.header.stamp;
  for (const auto &button_idx : m_button_map) {
    auto idx = button_idx.second;
    if (idx < msg.buttons.size()) {
      if (1 == msg.buttons[idx]) {
        res = res || handle_active_button(button_idx.first);
      }
    }
  }
  return res;
}

template <>
JoystickInterface::RawControl JoystickInterface::compute_command(
    const sensor_msgs::msg::Joy &msg) {
  RawControl res{};
  res.stamp = msg.header.stamp;
  axis_value(msg, Axes::LEFT_CMD, res.left_cmd);
  axis_value(msg, Axes::RIGHT_CMD, res.right_cmd);
  return res;
}

void JoystickInterface::reset_recordplay() {
  m_recordreplay_command.data =
      static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::NOOP);
}

const std_msgs::msg::UInt8 &JoystickInterface::get_recordreplay_command() {
  return m_recordreplay_command;
}

bool JoystickInterface::handle_active_button(Buttons button) {
  bool res = true;
  switch (button) {
    case Buttons::RECORDREPLAY_START_RECORD:
      m_recordreplay_command.data =
          static_cast<decltype(std_msgs::msg::UInt8::data)>(
              Recordreplay::START_RECORD);
      break;
    case Buttons::RECORDREPLAY_START_REPLAY:
      m_recordreplay_command.data =
          static_cast<decltype(std_msgs::msg::UInt8::data)>(
              Recordreplay::START_REPLAY);
      break;
    case Buttons::RECORDREPLAY_STOP:
      m_recordreplay_command.data =
          static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::STOP);
      break;
    default:
      throw std::logic_error{"Impossible button was pressed"};
  }
  return res;
}

template <typename T>
void JoystickInterface::axis_value(const sensor_msgs::msg::Joy &msg, Axes axis,
                                   T &value) const {
  const auto axis_iter = m_axis_map.find(axis);
  if (m_axis_map.end() == axis_iter) {
    return;
  }
  const auto axis_idx = axis_iter->second;
  if (axis_idx >= msg.axes.size()) {
    return;
  }
  const auto scale_iter = m_axis_scale_map.find(axis);
  const auto scale =
      scale_iter == m_axis_scale_map.end() ? DEFAULT_SCALE : scale_iter->second;
  const auto val_row = msg.axes[axis_idx] * scale;
  const auto offset_iter = m_axis_offset_map.find(axis);
  const auto offset = offset_iter == m_axis_offset_map.end()
                          ? DEFAULT_OFFSET
                          : offset_iter->second;
  using ValT = std::decay_t<decltype(value)>;
  value = static_cast<ValT>(val_row + offset);
}

const VehicleStateCommand &JoystickInterface::get_state_command() {
  return m_state_command;
}

}  // namespace joystick_interface