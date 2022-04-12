#include "vehicle_interface/platform_interface.hpp"

#include <iostream>

#include <rclcpp/logging.hpp>

namespace usv
{
namespace drivers
{
namespace vehicle_interface
{

PlatformInterface::PlatformInterface(
  const std::string & left_thruster_usb_name, const std::string & right_thruster_usb_name)
: m_ios{new io_service{}},
  m_work{new io_service::work{ios()}},
  m_left_usb{left_thruster_usb_name, serial_port{ios()}},
  m_right_usb{right_thruster_usb_name, serial_port{ios()}}
{
  m_left_usb.buffer.resize(REPORT_DATA_BYTES * 10);
  m_right_usb.buffer.resize(REPORT_DATA_BYTES * 10);
  m_left_usb.usb_port.open(left_thruster_usb_name);
  m_left_usb.usb_port.set_option(serial_port_base::baud_rate(DEFAULT_BAUD_RATE));
  m_right_usb.usb_port.open(right_thruster_usb_name);
  m_right_usb.usb_port.set_option(serial_port_base::baud_rate(DEFAULT_BAUD_RATE));
}

bool8_t PlatformInterface::send_control_command(const VehicleControlCommand & msg)
{
  auto left_ready = bool8_t{false};
  auto right_ready = bool8_t{false};
  while (!left_ready && !right_ready) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("485 receiver"), "not in normal phase");
    auto res = normal_phase_receive();
    left_ready = std::get<0>(res);
    right_ready = std::get<1>(res);
  }

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

  using boost::asio::buffer;
  if (left_ready) {
    const auto left_has_sent =
      m_left_usb.usb_port.write_some(buffer(left_to_send.data(), left_to_send.size()));
    if (left_has_sent != THROTTLE_DATA_BYTES) {
      return false;
    }
    if (msg.left_cmd >= 0) {
      m_left_direction = RotateDirection::FORWARD;
    } else {
      m_left_direction = RotateDirection::BACKWARD;
    }
  }
  if (right_ready) {
    const auto right_has_sent =
      m_right_usb.usb_port.write_some(buffer(right_to_send.data(), right_to_send.size()));
    if (right_has_sent != THROTTLE_DATA_BYTES) {
      return false;
    }
    if (msg.right_cmd >= 0) {
      m_right_direction = RotateDirection::FORWARD;
    } else {
      m_right_direction = RotateDirection::BACKWARD;
    }
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
  return m_left_usb.port_name;
}

std::string PlatformInterface::get_right_usb_port_name() const noexcept
{
  return m_right_usb.port_name;
}

PlatformInterface::~PlatformInterface()
{
  waitForExit();
  m_left_usb.usb_port.close();
  m_right_usb.usb_port.close();
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

std::experimental::optional<MotorReport1> PlatformInterface::get_left_motor_report_1()
  const noexcept
{
  return m_left_motor_report1;
}

std::experimental::optional<MotorReport2> PlatformInterface::get_left_motor_report_2()
  const noexcept
{
  return m_left_motor_report2;
}

std::experimental::optional<MotorReport1> PlatformInterface::get_right_motor_report_1()
  const noexcept
{
  return m_right_motor_report1;
}

std::experimental::optional<MotorReport2> PlatformInterface::get_right_motor_report_2()
  const noexcept
{
  return m_right_motor_report2;
}

std::tuple<bool8_t, bool8_t> PlatformInterface::normal_phase_receive()
{
  using boost::asio::read;
  auto left_has_received = read(
    m_left_usb.usb_port,
    mutable_buffer(m_left_usb.buffer.data(), m_left_usb.buffer.size()),
    boost::asio::transfer_at_least(2 * LEAST_RECEIVED_DATA_BYTES));
  auto right_has_received = read(
    m_right_usb.usb_port,
    mutable_buffer(m_right_usb.buffer.data(), m_right_usb.buffer.size()),
    boost::asio::transfer_at_least(2 * LEAST_RECEIVED_DATA_BYTES));

  auto left_cmd_type = uint8_t{};
  auto right_cmd_type = uint8_t{};

  const auto left_check =
    check_valid(m_left_usb, static_cast<sizeT>(left_has_received), left_cmd_type);
  const auto right_check =
    check_valid(m_right_usb, static_cast<sizeT>(right_has_received), right_cmd_type);

  if (!left_check && !right_check) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("485 receiver"), "no valid data in this read");
    return std::make_tuple(false, false);
  }

  bool8_t left_normal = (left_cmd_type == MOTOR_REPORT_CMD1 || left_cmd_type == MOTOR_REPORT_CMD2);
  bool8_t right_normal =
    (right_cmd_type == MOTOR_REPORT_CMD1 || right_cmd_type == MOTOR_REPORT_CMD2);

  if (left_normal || right_normal) {
    if (left_cmd_type == MOTOR_REPORT_CMD1) {
      update_motor_report1(m_left_usb, USBPorts::LEFT);
    } else if (left_cmd_type == MOTOR_REPORT_CMD2) {
      update_motor_report2(m_right_usb, USBPorts::LEFT);
    }
    if (right_cmd_type == MOTOR_REPORT_CMD1) {
      update_motor_report1(m_right_usb, USBPorts::RIGHT);
    } else if (right_cmd_type == MOTOR_REPORT_CMD2) {
      update_motor_report2(m_right_usb, USBPorts::RIGHT);
    }
    return std::make_tuple(left_normal, right_normal);
  }

  if (left_cmd_type == 0x22) {
    answer_22(USBPorts::LEFT);
  } else if (left_cmd_type == 0x26) {
    answer_26(USBPorts::LEFT);
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("485 receiver"), "unknown left_cmd_type");
  }

  if (right_cmd_type == 0x22) {
    answer_22(USBPorts::RIGHT);
  } else if (right_cmd_type == 0x26) {
    answer_26(USBPorts::RIGHT);
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("485 receiver"), "unknown right_cmd_type");
  }
  return std::make_tuple(false, false);
}

bool8_t PlatformInterface::answer_22(USBPorts port)
{
  using boost::asio::buffer;
  auto buf = buffer(ANSWER_22.data(), ANSWER_22.size());
  size_t has_send = 0;
  if (port == USBPorts::LEFT) {
    has_send = m_left_usb.usb_port.write_some(buf);
  } else if (port == USBPorts::RIGHT) {
    has_send = m_right_usb.usb_port.write_some(buf);
  }
  return has_send == ANSWER_22.size();
}

bool8_t PlatformInterface::answer_26(USBPorts port)
{
  using boost::asio::buffer;
  auto buf = buffer(ANSWER_26.data(), ANSWER_26.size());
  size_t has_send = 0;
  if (port == USBPorts::LEFT) {
    has_send = m_left_usb.usb_port.write_some(buf);
  } else if (port == USBPorts::RIGHT) {
    has_send = m_right_usb.usb_port.write_some(buf);
  }
  return has_send == ANSWER_26.size();
}

void PlatformInterface::update_motor_report1(UsbPortStruct & port, USBPorts port_name)
{
  auto cbegin = port.valid_begin;
  auto cend = port.valid_end;
  if (cend != cbegin + 12) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("485 receiver"), "wrong report1 data_len");
    return;
  }
  usv_msgs::msg::MotorReport1 msg{};
  msg.stamp = get_time_stamp();
  {
    // data 0
    if (((*cbegin) & 0x80) != 0) msg.stall = true;
    if (((*cbegin) & 0x40) != 0) msg.motor_over_temperature = true;
    if (((*cbegin) & 0x20) != 0) msg.mos_over_temperature = true;
    if (((*cbegin) & 0x10) != 0) msg.overcurrent = true;
    if (((*cbegin) & 0x08) != 0) msg.fault_8031 = true;
    if (((*cbegin) & 0x04) != 0) msg.communication_failure = true;
    if (((*cbegin) & 0x02) != 0) msg.motor_temperature_error = true;
    if (((*cbegin) & 0x01) != 0) msg.mos_temperature_alarm = true;
  }
  {
    // data 1
    if (((*(cbegin + 1)) & 0x80) != 0) msg.overvoltage = true;
    if (((*(cbegin + 1)) & 0x40) != 0) msg.undervoltage = true;
    if (((*(cbegin + 1)) & 0x20) != 0) msg.circuit_failure = true;
    if (((*(cbegin + 1)) & 0x10) != 0) msg.charge = true;
    if (((*(cbegin + 1)) & 0x08) != 0) msg.fan_failure = true;
  }
  {
    // data 2 and data 3
    using powerT = std::decay_t<decltype(msg.motor_power)>;
    msg.motor_power = static_cast<powerT>(
      (static_cast<uint16_t>(*(cbegin + 2)) << 8) + static_cast<uint16_t>(*(cbegin + 3)));
  }
  {
    // data 4 and data 5
    using voltageT = std::decay_t<decltype(msg.voltage)>;
    msg.voltage =
      0.1f * static_cast<voltageT>(
               (static_cast<uint16_t>(*(cbegin + 4)) << 8) + static_cast<uint16_t>(*(cbegin + 5)));
  }
  {
    // data 6 and data 7
    using rpmT = std::decay_t<decltype(msg.rotating_speed)>;
    msg.rotating_speed = static_cast<rpmT>(
      (static_cast<uint16_t>(*(cbegin + 6)) << 8) + static_cast<uint16_t>(*(cbegin + 7)));
  }
  {
    // data 8 and data 9
    using phaseT = std::decay_t<decltype(msg.phase_current)>;
    msg.phase_current = static_cast<phaseT>(
      (static_cast<uint16_t>(*(cbegin + 8)) << 8) + static_cast<uint16_t>(*(cbegin + 9)));
  }
  {
    // data 10 and data 11
    // TODO check uint -> int
    using temT = std::decay_t<decltype(msg.motor_temperature)>;
    msg.motor_temperature = static_cast<temT>(
      (static_cast<uint16_t>(*(cbegin + 10)) << 8) + static_cast<uint16_t>(*(cbegin + 11)));
  }

  if (USBPorts::LEFT == port_name) {
    if (RotateDirection::FORWARD == m_left_direction) {
      msg.rotate_direction = MotorReport1::FORWARD;
    } else if (RotateDirection::BACKWARD == m_left_direction) {
      msg.rotate_direction = MotorReport1::BACKWARD;
    }
    m_left_motor_report1.emplace(msg);
    m_left_direction = RotateDirection::NOT_SET;
  } else if (USBPorts::RIGHT == port_name) {
    if (RotateDirection::FORWARD == m_right_direction) {
      msg.rotate_direction = MotorReport1::FORWARD;
    } else if (RotateDirection::BACKWARD == m_right_direction) {
      msg.rotate_direction = MotorReport1::BACKWARD;
    }
    m_right_motor_report1.emplace(msg);
    m_right_direction = RotateDirection::NOT_SET;
  }
}

void PlatformInterface::update_motor_report2(UsbPortStruct & port, USBPorts port_name)
{
  auto cbegin = port.valid_begin;
  auto cend = port.valid_end;
  if (cend != cbegin + 12) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("485 receiver"), "wrong report2 data_len");
    return;
  }
  usv_msgs::msg::MotorReport2 msg{};
  msg.stamp = get_time_stamp();
  {
    // data0 and data 1
    // TODO check uint -> int
    using mosTemerature_T = std::decay_t<decltype(msg.mos_temperature)>;
    msg.mos_temperature = static_cast<mosTemerature_T>(
      (static_cast<uint16_t>(*cbegin) << 8) + static_cast<uint16_t>(*(cbegin + 1)));
  }
  {
    // data 2 and data 3
    // TODO check uint -> int
    using batteryTem_T = std::decay_t<decltype(msg.battery_temperature)>;
    msg.battery_temperature = static_cast<batteryTem_T>(
      (static_cast<uint16_t>(*(cbegin + 2)) << 8) + static_cast<uint16_t>(*(cbegin + 3)));
  }
  {
    // data 4 and data 5
    using busCurrent_T = std::decay_t<decltype(msg.bus_current)>;
    msg.bus_current = static_cast<busCurrent_T>(
      (static_cast<uint16_t>(*(cbegin + 4)) << 8) + static_cast<uint16_t>(*(cbegin + 5)));
  }
  {
    // data 6 and data 7
    using singleRunTime_T = std::decay_t<decltype(msg.single_run_time)>;
    msg.single_run_time = static_cast<singleRunTime_T>(
      (static_cast<uint16_t>(*(cbegin + 6)) << 8) + static_cast<uint16_t>(*(cbegin + 7)));
  }
  {
    // data 8 and data 9
    using totalRunTime_T = std::decay_t<decltype(msg.total_run_time)>;
    msg.total_run_time = static_cast<totalRunTime_T>(
      (static_cast<uint16_t>(*(cbegin + 8)) << 8) + static_cast<uint16_t>(*(cbegin + 9)));
  }
  {
    // data 10 and data 11
    using reChargeTime_T = std::decay_t<decltype(msg.recharge_time)>;
    msg.recharge_time = static_cast<reChargeTime_T>(
      (static_cast<uint16_t>(*(cbegin + 10)) << 8) + static_cast<uint16_t>(*(cbegin + 11)));
  }
  if (USBPorts::LEFT == port_name) {
    m_left_motor_report2.emplace(msg);
  } else if (USBPorts::RIGHT == port_name) {
    m_right_motor_report2.emplace(msg);
  }
}

void PlatformInterface::reset_left_motor_report1() noexcept
{
  m_left_motor_report1 = std::experimental::optional<MotorReport1>{};
}
void PlatformInterface::reset_left_motor_report2() noexcept
{
  m_left_motor_report2 = std::experimental::optional<MotorReport2>{};
}

void PlatformInterface::reset_right_motor_report1() noexcept
{
  m_right_motor_report1 = std::experimental::optional<MotorReport1>{};
}

void PlatformInterface::reset_right_motor_report2() noexcept
{
  m_right_motor_report2 = std::experimental::optional<MotorReport2>{};
}

builtin_interfaces::msg::Time PlatformInterface::get_time_stamp()
{
  using time_utils::to_message;
  return to_message(std::chrono::system_clock::now());
}

bool8_t PlatformInterface::check_valid(UsbPortStruct & port, sizeT received_len, uint8_t & cmd_type)
{
  auto ptr = port.buffer.cbegin();
  const auto end = ptr + received_len;
  auto valid_end = std::experimental::optional<iterT>{};
  bool8_t valid = false;
  while (!valid) {
    ptr = std::find(ptr, end, HEAD);
    if (ptr > end - LEAST_RECEIVED_DATA_BYTES) {
      break;
    }
    ptr += 1;
    if (ptr != end && *ptr != ADDRESS) continue;
    auto data_len = static_cast<sizeT>(*(ptr + 1));
    if (ptr + 3 + data_len < end && *(ptr + 3 + data_len) == END) {
      auto xor_check = *(ptr + 1);
      for (sizeT i = 0; i < data_len; ++i) {
        xor_check ^= (*(ptr + 2 + i));
      }
      if (xor_check == *(ptr + 2 + data_len)) {
        valid = true;
        valid_end.emplace(ptr + 2 + data_len);
      }
    }
  }
  if (valid) {
    cmd_type = *(ptr + 2);
    port.valid_begin = ptr + 3;
    port.valid_end = valid_end.value();
  }
  return valid;
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv
