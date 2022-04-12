#ifndef VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
#define VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_

#include <chrono>
#include <experimental/optional>

#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <common/types.hpp>
#include <time_utils/time_utils.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <usv_msgs/msg/high_level_control_command.hpp>
#include <usv_msgs/msg/motor_report1.hpp>
#include <usv_msgs/msg/motor_report2.hpp>
#include <usv_msgs/msg/vehicle_control_command.hpp>

using usv::common::types::bool8_t;

using usv_msgs::msg::HighLevelControlCommand;
using usv_msgs::msg::MotorReport1;
using usv_msgs::msg::MotorReport2;
using usv_msgs::msg::VehicleControlCommand;

using boost::asio::io_service;
using boost::asio::mutable_buffer;
using boost::asio::serial_port;
using boost::asio::serial_port_base;

namespace usv
{
namespace drivers
{
namespace vehicle_interface
{
constexpr size_t THROTTLE_DATA_BYTES = 8;
constexpr size_t REPORT_DATA_BYTES = 18;
constexpr size_t LEAST_RECEIVED_DATA_BYTES = 14;
// 8 Bytes: head address length cmd data0 data1 check end
constexpr uint8_t HEAD = 0x28;
constexpr uint8_t ADDRESS = 0x04;
constexpr uint8_t THROTTLE_DATA_LEN = 0x03;
constexpr uint8_t THROTTLE_CMD = 0x40;
constexpr uint8_t MOTOR_REPORT_CMD1 = 0x25;
constexpr uint8_t MOTOR_REPORT_CMD2 = 0x27;
constexpr uint8_t END = 0x29;
constexpr uint32_t DEFAULT_BAUD_RATE = 38400;

constexpr auto ANSWER_22 = std::array<uint8_t, 7>{HEAD, ADDRESS, 0x02, 0x44, 0x22, 0x64, END};
constexpr auto ANSWER_26 = std::array<uint8_t, 7>{HEAD, ADDRESS, 0x02, 0x44, 0x26, 0x60, END};

enum class USBPorts { LEFT, RIGHT };
enum class RotateDirection { FORWARD, BACKWARD, NOT_SET };

struct VEHICLE_INTERFACE_LOCAL UsbPortStruct {
  using bufferT = std::vector<uint8_t>;
  using iterT = bufferT::const_iterator;
  std::string port_name;
  serial_port usb_port;
  bufferT buffer{};
  iterT valid_begin{nullptr};
  iterT valid_end{nullptr};
};

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
  // not implemented yet
  static bool8_t send_control_command(const HighLevelControlCommand & msg);

  std::string get_left_usb_port_name() const noexcept;
  std::string get_right_usb_port_name() const noexcept;

  std::experimental::optional<MotorReport1> get_left_motor_report_1() const noexcept;
  std::experimental::optional<MotorReport2> get_left_motor_report_2() const noexcept;
  std::experimental::optional<MotorReport1> get_right_motor_report_1() const noexcept;
  std::experimental::optional<MotorReport2> get_right_motor_report_2() const noexcept;

  void reset_left_motor_report1() noexcept;
  void reset_left_motor_report2() noexcept;
  void reset_right_motor_report1() noexcept;
  void reset_right_motor_report2() noexcept;

  std::tuple<bool8_t, bool8_t> normal_phase_receive();

  io_service & ios() const;
  void waitForExit();

  using iterT = UsbPortStruct::iterT;
  using sizeT = iterT::difference_type;

  bool8_t check_valid(UsbPortStruct& port, sizeT received_len, uint8_t& cmd_type);
  bool8_t answer_22(USBPorts port);
  bool8_t answer_26(USBPorts port);
  void update_motor_report1(UsbPortStruct& port, USBPorts port_name);
  void update_motor_report2(UsbPortStruct& port, USBPorts port_name);

private:
  static builtin_interfaces::msg::Time get_time_stamp();

  std::shared_ptr<io_service> m_ios;
  std::shared_ptr<io_service::work> m_work;

  std::experimental::optional<MotorReport1> m_left_motor_report1{};
  std::experimental::optional<MotorReport2> m_left_motor_report2{};
  std::experimental::optional<MotorReport1> m_right_motor_report1{};
  std::experimental::optional<MotorReport2> m_right_motor_report2{};

  // the feedback from motor not including the direction, need to preserve
  RotateDirection m_left_direction{RotateDirection::NOT_SET};
  RotateDirection m_right_direction{RotateDirection::NOT_SET};

  UsbPortStruct m_left_usb;
  UsbPortStruct m_right_usb;
};  // class PlatformInterface
}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv

#endif  // VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
