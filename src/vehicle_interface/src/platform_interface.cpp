#include "vehicle_interface/platform_interface.hpp"

namespace usv
{
namespace drivers
{
namespace vehicle_interface
{

PlatformInterface::PlatformInterface(
  const std::string & left_thruster_usb_name, const std::string & right_thruster_usb_name)
: m_left_thruster_usb_name{left_thruster_usb_name},
  m_right_thruster_usb_name{right_thruster_usb_name},
  m_left_usb{
    IoContext{},
    left_thruster_usb_name,
    {DEFAULT_BAUD_RATE, FlowControl::NONE, Parity::NONE, StopBits::ONE}},
  m_right_usb{
    IoContext{},
    right_thruster_usb_name,
    {DEFAULT_BAUD_RATE, FlowControl::NONE, Parity::NONE, StopBits::ONE}}
{
}

bool8_t PlatformInterface::send_control_command(const VehicleControlCommand & msg)
{
  auto data0 = [](auto & cmd) -> uint8_t { return 0 <= cmd ? 0x01 : 0x00; };
  auto data1 = [](auto cmd) -> uint8_t {
    if (0 > cmd) cmd = -cmd;
    return static_cast<uint8_t>(round(static_cast<double>(cmd)));
  };
  auto xor_check = [](auto & data0, auto & data1) -> uint8_t {
    return THROTTLE_DATA_LEN ^ THROTTLE_CMD ^ data0 ^ data1;
  };

  auto left_data0 = data0(msg.left_cmd);
  auto right_data0 = data0(msg.right_cmd);
  auto left_data1 = data1(msg.left_cmd);
  auto right_data1 = data1(msg.right_cmd);
  auto left_xor = xor_check(left_data0, left_data1);
  auto right_xor = xor_check(right_data0, right_data1);

  auto data_to_send = [](auto & data0, auto & data1, auto & xor_check) -> std::vector<uint8_t> {
    return std::vector<uint8_t>{
      HEAD, ADDRESS, THROTTLE_DATA_LEN, THROTTLE_CMD, data0, data1, xor_check, END};
  };

  auto left_to_send = data_to_send(left_data0, left_data1, left_xor);
  auto right_to_send = data_to_send(right_data0, right_data1, right_xor);

  auto left_has_sent = m_left_usb.send(left_to_send);
  auto right_has_sent = m_right_usb.send(right_to_send);
  if (left_has_sent != THROTTLE_DATA_BYTES || right_has_sent != THROTTLE_DATA_BYTES) {
    return false;
  }
  return true;
}

bool8_t PlatformInterface::send_control_command(const RawControlCommand &)
{
  // not implemented yet;
  return true;
}

std::string PlatformInterface::get_left_usb_port_name() const noexcept
{
  return m_left_thruster_usb_name;
}

std::string PlatformInterface::get_right_usb_port_name() const noexcept
{
  return m_right_thruster_usb_name;
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv
