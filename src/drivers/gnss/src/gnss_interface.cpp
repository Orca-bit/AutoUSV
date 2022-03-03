//
// Created by liuhao on 2022/3/2.
//

#include "gnss/gnss_interface.hpp"

#include <cmath>
#include <sstream>
#include <utility>

namespace usv
{
namespace drivers
{

std::vector<std::string> gnss::GnssInterface::split(const std::string & str)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(str);

  while (getline(ss, token, ',')) str_vec_ptr.push_back(token);

  return std::move(str_vec_ptr);
}

void gnss::GnssInterface::convert(const std::vector<std::string> & nmea)
{
  if (nmea.at(0) == "$GPFPD") {
    // north 0 degree, east 90 degree
    const auto heading = stod(nmea.at(3)) * M_PI / 180.;
    const auto sin_heading = sin(heading);
    const auto cos_heading = cos(heading);
    const auto ve = stod(nmea.at(9));
    const auto vn = stod(nmea.at(10));
    // convert velocity to body fix frame
    const auto u = ve * sin_heading + vn * cos_heading;
    const auto v = vn * sin_heading - ve * cos_heading;
    {
      using T = decltype(m_state.state.longitudinal_velocity_mps);
      m_state.state.longitudinal_velocity_mps = static_cast<T>(u);
      m_state.state.lateral_velocity_mps = static_cast<T>(v);
      m_linear_velocity_ready = true;
      m_linear_velocity_received_time = std::chrono::system_clock::now();
    }
  } else if (nmea.at(0) == "$GTIMU") {
    /// TODO: check direction
    const auto r = stod(nmea.at(5)) * M_PI / 180.;
    {
      using T = decltype(m_state.state.heading_rate_rps);
      m_state.state.heading_rate_rps = static_cast<T>(r);
      m_angular_velocity_ready = true;
      m_angular_velocity_received_time = std::chrono::system_clock::now();
    }
  } else {
    throw std::logic_error{"only GPFPD and GTIMU are supported"};
  }
}

boost::asio::io_service & gnss::GnssInterface::ios() const
{
  return *m_ios;
}

void gnss::GnssInterface::waitForExit()
{
  if (!ios().stopped()) {
    ios().post([&]() { m_work.reset(); });
  }
  ios().stop();
}

gnss::GnssInterface::GnssInterface(std::string usb_port)
: m_usb_port_name{std::move(usb_port)},
  m_ios{new io_service{}},
  m_work{new io_service::work{ios()}},
  m_port{ios()}
{
  m_port.open(m_usb_port_name);
  m_port.set_option(serial_port_base::baud_rate(115200));
}

gnss::GnssInterface::~GnssInterface()
{
  waitForExit();
  m_port.close();
}

gnss::State gnss::GnssInterface::work()
{
  while (!(m_linear_velocity_ready && m_angular_velocity_ready)) {
    read_data();
    std::string str{buffer};
    convert(split(str.substr(str.find('$'))));
    const std::chrono::duration<double> diff =
      std::max(m_linear_velocity_received_time, m_angular_velocity_received_time) -
      std::min(m_linear_velocity_received_time, m_angular_velocity_received_time);
    if (diff.count() > 5e-2) {
      m_linear_velocity_ready = false;
      m_angular_velocity_ready = false;
    }
  }

  return m_state;
}

void gnss::GnssInterface::read_data()
{
  using boost::asio::read;
  read(m_port, mutable_buffer(buffer, 512), boost::asio::transfer_at_least(256));
}

std::string gnss::GnssInterface::get_port_name() const noexcept
{
  return m_usb_port_name;
}

}  // namespace drivers
}  // namespace usv