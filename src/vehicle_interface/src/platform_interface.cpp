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
  m_ios{new io_service{}},
  m_work{new io_service::work{ios()}},
  m_left_usb{ios()},
  m_right_usb{ios()}
{
  m_left_usb.open(left_thruster_usb_name);
  m_right_usb.open(right_thruster_usb_name);
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

  const auto left_data0 = data0(msg.left_cmd);
  const auto right_data0 = data0(msg.right_cmd);
  const auto left_data1 = data1(msg.left_cmd);
  const auto right_data1 = data1(msg.right_cmd);
  const auto left_xor = xor_check(left_data0, left_data1);
  const auto right_xor = xor_check(right_data0, right_data1);

  auto data_to_send = [](auto & data0, auto & data1, auto & xor_check) -> std::vector<uint8_t> {
    return std::vector<uint8_t>{
      HEAD, ADDRESS, THROTTLE_DATA_LEN, THROTTLE_CMD, data0, data1, xor_check, END};
  };

  const auto left_to_send = data_to_send(left_data0, left_data1, left_xor);
  const auto right_to_send = data_to_send(right_data0, right_data1, right_xor);

  const auto left_has_sent =
    m_left_usb.write_some(buffer(left_to_send.data(), left_to_send.size()));
  const auto right_has_sent =
    m_right_usb.write_some(buffer(right_to_send.data(), right_to_send.size()));

  if (left_has_sent != THROTTLE_DATA_BYTES || right_has_sent != THROTTLE_DATA_BYTES) {
    return false;
  }
  return true;
}

/// TODO: implement
bool8_t PlatformInterface::send_control_command(const HighLevelControlCommand &)
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

PlatformInterface::~PlatformInterface()
{
  waitForExit();
  m_left_usb.close();
  m_right_usb.close();
}

void PlatformInterface::waitForExit()
{
  if (!ios().stopped()) {
    ios().post([&]() { m_work.reset(); });
  }
  ios().stop();
}

io_service & PlatformInterface::ios() const
{
  return *m_ios;
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv
