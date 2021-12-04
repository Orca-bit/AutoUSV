#ifndef VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
#define VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_

#include <chrono>

#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <common/types.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <usv_msgs/msg/high_level_control_command.hpp>
#include <usv_msgs/msg/vehicle_control_command.hpp>

using usv::common::types::bool8_t;

using usv_msgs::msg::HighLevelControlCommand;
using usv_msgs::msg::VehicleControlCommand;

using boost::asio::buffer;
using boost::asio::io_service;
using boost::asio::serial_port;
using boost::asio::serial_port_base;

namespace usv
{
namespace drivers
{
namespace vehicle_interface
{

constexpr size_t THROTTLE_DATA_BYTES = 8;
// 8 Bytes: head address length cmd data0 data1 check end
constexpr uint8_t HEAD = 0x28;
constexpr uint8_t ADDRESS = 0x04;
constexpr uint8_t THROTTLE_DATA_LEN = 0x03;
constexpr uint8_t THROTTLE_CMD = 0x40;
constexpr uint8_t END = 0x29;
constexpr uint32_t DEFAULT_BAUD_RATE = 38400;

/// Interface class for specific vehicle implementations. Child classes which implement this
/// interface are expected to have wrap their own communication mechanism, and create a subclass
/// from the VehicleInterfaceNode
class VEHICLE_INTERFACE_PUBLIC PlatformInterface
{
public:
  /// Constructor
  PlatformInterface(
    const std::string & left_thruster_usb_name, const std::string & right_thruster_usb_name);
  /// Destructor
  ~PlatformInterface();
  // copy forbidden
  PlatformInterface(const PlatformInterface &) = delete;
  PlatformInterface & operator=(const PlatformInterface &) = delete;

  bool8_t send_control_command(const VehicleControlCommand & msg);
  bool8_t send_control_command(const HighLevelControlCommand & msg);

  std::string get_left_usb_port_name() const noexcept;
  std::string get_right_usb_port_name() const noexcept;

  io_service & ios() const;
  void waitForExit();

private:
  std::string m_left_thruster_usb_name;
  std::string m_right_thruster_usb_name;

  std::shared_ptr<io_service> m_ios;
  std::shared_ptr<io_service::work> m_work;

  serial_port m_left_usb;
  serial_port m_right_usb;
};  // class PlatformInterface
}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv

#endif  // VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
