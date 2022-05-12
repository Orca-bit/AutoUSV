//
// Created by liuhao on 2022/3/2.
//

#include "gnss/gnss_interface.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <sstream>
#include <utility>

#include <time_utils/time_utils.hpp>

namespace usv
{
namespace drivers
{

std::vector<std::string> gnss::GnssInterface::split(const std::string & str)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::string line;
  std::istringstream ss{str};

  while (getline(ss, line)) {
    std::istringstream s{line};
    while (getline(s, token, ',')) str_vec_ptr.push_back(token);
  }

  return str_vec_ptr;
}

void gnss::GnssInterface::convert(const std::vector<std::string> & nmea)
{
  const auto iter_gpfpd = std::find(nmea.begin(), nmea.end(), "$GPFPD");
  const auto iter_gtimu = std::find(nmea.begin(), nmea.end(), "$GTIMU");
  if (std::distance(iter_gpfpd, nmea.end()) > 11) {
    // from: north 0 degree, east 90 degree, 0~360 degrees
    // to: east 0 rad, north pi/2 rad, -pi~pi rad, i.e. right hands
    // first to: east 0 rad, north pi/2 rad, -3pi/2 ~ pi/2
    auto heading = M_PI / 2. - stod(*(iter_gpfpd + 3)) * M_PI / 180.;
    // second: change range of -3pi/2 ~ -pi to pi/2 ~ pi
    if (heading < -M_PI) {
      heading += 2 * M_PI;
    }
    // need change pitch direction
    const auto pitch = -stod(*(iter_gpfpd + 4));
    const auto roll = stod(*(iter_gpfpd + 5));
    const auto lat_deg = stod(*(iter_gpfpd + 6));
    const auto lon_deg = stod(*(iter_gpfpd + 7));
    const auto h = stod(*(iter_gpfpd + 8));
    {
      if (!m_conv->check_plane_setting()) {
        m_conv->set_plane_degree(lon_deg, lat_deg);
      }
      m_conv->set_llh_degree(lon_deg, lat_deg, h);
      m_state.header.frame_id = "odom";
      m_state.state.pose.position.x = m_conv->x();
      m_state.state.pose.position.y = m_conv->y();
      m_state.state.pose.position.z = m_conv->z();

      tf2::Quaternion quaternion_tf;
      quaternion_tf.setRPY(roll, pitch, heading);
      m_state.state.pose.orientation = tf2::toMsg(quaternion_tf);
    }
    const auto sin_heading = sin(heading);
    const auto cos_heading = cos(heading);
    const auto ve = stod(*(iter_gpfpd + 9));
    const auto vn = stod(*(iter_gpfpd + 10));
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
  }
  if (std::distance(iter_gtimu, nmea.end()) > 6) {
    /// TODO: check direction
    const auto r = stod(*(iter_gtimu + 5)) * M_PI / 180.;
    {
      using T = decltype(m_state.state.heading_rate_rps);
      m_state.state.heading_rate_rps = static_cast<T>(r);
      m_angular_velocity_ready = true;
      m_angular_velocity_received_time = std::chrono::system_clock::now();
    }
  }
  if (iter_gtimu == nmea.end() && iter_gpfpd == nmea.end()) {
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

gnss::GnssInterface::GnssInterface(const std::string & usb_port)
: m_ios{new io_service{}},
  m_work{new io_service::work{ios()}},
  m_port{ios()},
  m_usb_port_name{usb_port}
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
    auto s = str.substr(str.find('$'));
    convert(split(s));
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
  m_linear_velocity_ready = false;
  m_angular_velocity_ready = false;
  return m_state;
}

void gnss::GnssInterface::read_data()
{
  using boost::asio::read;
  read(m_port, mutable_buffer(m_buffer, 256), boost::asio::transfer_at_least(256));
}

std::string gnss::GnssInterface::get_port_name() const noexcept
{
  return m_usb_port_name;
}


}  // namespace drivers
}  // namespace usv