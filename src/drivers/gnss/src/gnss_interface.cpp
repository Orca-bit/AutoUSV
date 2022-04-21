//
// Created by liuhao on 2022/3/2.
//

#include "gnss/gnss_interface.hpp"

#include <time_utils/time_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

  return str_vec_ptr;
}

void gnss::GnssInterface::convert(const std::vector<std::string> & nmea)
{
  if (nmea.at(0) == "$GPFPD") {
    // from: north 0 degree, east 90 degree, 0~360 degrees
    // to: east 0 rad, north pi/2 rad, -pi~pi rad, i.e. right hands
    // first to: east 0 rad, north pi/2 rad, -3pi/2 ~ pi/2
    auto heading =  M_PI / 2. - stod(nmea.at(3)) * M_PI / 180.;
    // second: change range of -3pi/2 ~ -pi to pi/2 ~ pi
    if (heading < -M_PI) {
      heading += 2 * M_PI;
    }
    // need change pitch direction
    const auto pitch = -stod(nmea.at(4));
    const auto roll = stod(nmea.at(5));
    const auto lat_deg = stod(nmea.at(6));
    const auto lon_deg = stod(nmea.at(7));
    const auto h = stod(nmea.at(8));
    {
      if (!m_conv->check_plane_setting()) {
        m_conv->set_plane_degree(lon_deg, lat_deg);
      }
      m_conv->set_llh_degree(lon_deg, lat_deg, h);
      m_state.header.frame_id = "map";
      m_state.state.pose.position.x = m_conv->x();
      m_state.state.pose.position.y = m_conv->y();
      m_state.state.pose.position.z = m_conv->z();

      tf2::Quaternion quaternion_tf;
      quaternion_tf.setRPY(roll, pitch, heading);
      m_state.state.pose.orientation = tf2::toMsg(quaternion_tf);
    }
    const auto sin_heading = sin(heading);
    const auto cos_heading = cos(heading);
    const auto ve = stod(nmea.at(9));
    const auto vn = stod(nmea.at(10));
    // convert velocity to body fix frame
    const auto u = ve * cos_heading + vn * sin_heading;
    const auto v = -ve * sin_heading + vn * cos_heading;
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
  m_port{ios()},
  m_ios{new io_service{}},
  m_work{new io_service::work{ios()}}
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
    std::string str{m_buffer};
    convert(split(str.substr(str.find('$'))));
    const std::chrono::duration<double> diff =
      std::max(m_linear_velocity_received_time, m_angular_velocity_received_time) -
      std::min(m_linear_velocity_received_time, m_angular_velocity_received_time);
    if (diff.count() > 1e-1) {
      m_linear_velocity_ready = false;
      m_angular_velocity_ready = false;
    }
  }

  using time_utils::to_message;
  m_state.header.stamp = to_message(std::chrono::system_clock::now());
  return m_state;
}

void gnss::GnssInterface::read_data()
{
  using boost::asio::read;
  read(m_port, mutable_buffer(m_buffer, 512), boost::asio::transfer_at_least(256));
}

std::string gnss::GnssInterface::get_port_name() const noexcept
{
  return m_usb_port_name;
}


}  // namespace drivers
}  // namespace usv