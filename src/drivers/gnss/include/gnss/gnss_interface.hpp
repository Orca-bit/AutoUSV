//
// Created by liuhao on 2022/3/2.
//

#ifndef GNSS_GNSS_INTERFACE_HPP
#define GNSS_GNSS_INTERFACE_HPP

#include <chrono>
#include <string>
#include <vector>

#include "gnss/visibility_control.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>

#include <usv_msgs/msg/vehicle_kinematic_state.hpp>

namespace usv
{
namespace drivers
{
namespace gnss
{
using boost::asio::io_service;
using boost::asio::mutable_buffer;
using boost::asio::serial_port;
using boost::asio::serial_port_base;

using State = usv_msgs::msg::VehicleKinematicState;

class GNSS_INTERFACE_PUBLIC GnssInterface
{
public:
  explicit GnssInterface(std::string usb_port);
  ~GnssInterface();

  GnssInterface(const GnssInterface &) = delete;
  GnssInterface & operator=(const GnssInterface &) = delete;
  GnssInterface(GnssInterface &&) = delete;
  GnssInterface & operator=(GnssInterface &&) = delete;

  State work();
  std::string get_port_name() const noexcept;

private:
  GNSS_INTERFACE_LOCAL io_service & ios() const;
  GNSS_INTERFACE_LOCAL void waitForExit();

  GNSS_INTERFACE_LOCAL void read_data();

  GNSS_INTERFACE_LOCAL std::vector<std::string> split(const std::string & str);
  GNSS_INTERFACE_LOCAL void convert(const std::vector<std::string> & nmea);

  State m_state{rosidl_runtime_cpp::MessageInitialization::ALL};
  bool m_linear_velocity_ready{false};
  bool m_angular_velocity_ready{false};

  std::chrono::time_point<std::chrono::system_clock> m_linear_velocity_received_time{};
  std::chrono::time_point<std::chrono::system_clock> m_angular_velocity_received_time{};

  std::string m_usb_port_name;
  serial_port m_port;
  char buffer[512]{};

  std::shared_ptr<io_service> m_ios;
  std::shared_ptr<io_service::work> m_work;
};
}  // namespace gnss
}  // namespace drivers
}  // namespace usv

#endif  // GNSS_GNSS_INTERFACE_HPP
