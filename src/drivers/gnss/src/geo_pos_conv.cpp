//
// Created by liuhao on 22-4-19.
//

#include "gnss/geo_pos_conv.hpp"

#include <cmath>

namespace usv
{
namespace drivers
{
namespace gnss
{

double GeoPosConv::x() const
{
  return m_x;
}
////////////////////////////////////////////////////////////////////////////

double GeoPosConv::y() const
{
  return m_y;
}
////////////////////////////////////////////////////////////////////////////

double GeoPosConv::z() const
{
  return m_z;
}
////////////////////////////////////////////////////////////////////////////

void GeoPosConv::set_plane_degree(double lon, double lat)
{
  m_plane_lon = lon * M_PI / 180.;
  m_plane_lat = lat * M_PI / 180.;
}
////////////////////////////////////////////////////////////////////////////

void GeoPosConv::set_plane_radian(double lon, double lat)
{
  m_plane_lon = lon;
  m_plane_lat = lat;
}
////////////////////////////////////////////////////////////////////////////

void GeoPosConv::set_llh_degree(double lon, double lat, double h)
{
  m_lon = lon * M_PI / 180.;
  m_lat = lat * M_PI / 180.;
  m_h = h;
  conv_llh2xyz();
}
////////////////////////////////////////////////////////////////////////////

void GeoPosConv::set_llh_radian(double lon, double lat, double h)
{
  m_lon = lon;
  m_lat = lat;
  m_h = h;
  conv_llh2xyz();
}
////////////////////////////////////////////////////////////////////////////

/// see
/// https://github.com/Autoware-AI/common/blob/9e2c61b3e610a389512ffbe10a01fb5a73598a92/gnss/src/GeoPosConv.cpp#L253
void GeoPosConv::conv_llh2xyz()
{
  const double Pmo = 0.9999;
  /*WGS84 Parameters*/
  const double AW = 6378137.0;
  const double FW = 1.0 / 298.257222101;

  const auto Pe = std::sqrt(2.0 * FW - std::pow(FW, 2));
  const auto Pet = std::sqrt(std::pow(Pe, 2) / (1.0 - std::pow(Pe, 2)));
  const auto PA = 1.0 + 3.0 / 4.0 * std::pow(Pe, 2) + 45.0 / 64.0 * std::pow(Pe, 4) +
                  175.0 / 256.0 * std::pow(Pe, 6) + 11025.0 / 16384.0 * std::pow(Pe, 8) +
                  43659.0 / 65536.0 * std::pow(Pe, 10) + 693693.0 / 1048576.0 * std::pow(Pe, 12) +
                  19324305.0 / 29360128.0 * std::pow(Pe, 14) +
                  4927697775.0 / 7516192768.0 * std::pow(Pe, 16);
  const auto PB = 3.0 / 4.0 * std::pow(Pe, 2) + 15.0 / 16.0 * std::pow(Pe, 4) +
                  525.0 / 512.0 * std::pow(Pe, 6) + 2205.0 / 2048.0 * std::pow(Pe, 8) +
                  72765.0 / 65536.0 * std::pow(Pe, 10) + 297297.0 / 262144.0 * std::pow(Pe, 12) +
                  135270135.0 / 117440512.0 * std::pow(Pe, 14) +
                  547521975.0 / 469762048.0 * std::pow(Pe, 16);
  const auto PC = 15.0 / 64.0 * std::pow(Pe, 4) + 105.0 / 256.0 * std::pow(Pe, 6) +
                  2205.0 / 4096.0 * std::pow(Pe, 8) + 10395.0 / 16384.0 * std::pow(Pe, 10) +
                  1486485.0 / 2097152.0 * std::pow(Pe, 12) +
                  45090045.0 / 58720256.0 * std::pow(Pe, 14) +
                  766530765.0 / 939524096.0 * std::pow(Pe, 16);
  const auto PD = 35.0 / 512.0 * std::pow(Pe, 6) + 315.0 / 2048.0 * std::pow(Pe, 8) +
                  31185.0 / 131072.0 * std::pow(Pe, 10) + 165165.0 / 524288.0 * std::pow(Pe, 12) +
                  45090045.0 / 117440512.0 * std::pow(Pe, 14) +
                  209053845.0 / 469762048.0 * std::pow(Pe, 16);
  const auto PE = 315.0 / 16384.0 * std::pow(Pe, 8) + 3465.0 / 65536.0 * std::pow(Pe, 10) +
                  99099.0 / 1048576.0 * std::pow(Pe, 12) +
                  4099095.0 / 29360128.0 * std::pow(Pe, 14) +
                  348423075.0 / 1879048192.0 * std::pow(Pe, 16);
  const auto PF = 693.0 / 131072.0 * std::pow(Pe, 10) + 9009.0 / 524288.0 * std::pow(Pe, 12) +
                  4099095.0 / 117440512.0 * std::pow(Pe, 14) +
                  26801775.0 / 469762048.0 * std::pow(Pe, 16);
  const auto PG = 3003.0 / 2097152.0 * std::pow(Pe, 12) + 315315.0 / 58720256.0 * std::pow(Pe, 14) +
                  11486475.0 / 939524096.0 * std::pow(Pe, 16);
  const auto PH =
    45045.0 / 117440512.0 * std::pow(Pe, 14) + 765765.0 / 469762048.0 * std::pow(Pe, 16);
  const auto PI = 765765.0 / 7516192768.0 * std::pow(Pe, 16);

  const auto PB1 = AW * (1.0 - std::pow(Pe, 2)) * PA;
  const auto PB2 = AW * (1.0 - std::pow(Pe, 2)) * PB / -2.0;
  const auto PB3 = AW * (1.0 - std::pow(Pe, 2)) * PC / 4.0;
  const auto PB4 = AW * (1.0 - std::pow(Pe, 2)) * PD / -6.0;
  const auto PB5 = AW * (1.0 - std::pow(Pe, 2)) * PE / 8.0;
  const auto PB6 = AW * (1.0 - std::pow(Pe, 2)) * PF / -10.0;
  const auto PB7 = AW * (1.0 - std::pow(Pe, 2)) * PG / 12.0;
  const auto PB8 = AW * (1.0 - std::pow(Pe, 2)) * PH / -14.0;
  const auto PB9 = AW * (1.0 - std::pow(Pe, 2)) * PI / 16.0;

  const auto PS = PB1 * m_lat + PB2 * std::sin(2.0 * m_lat) + PB3 * std::sin(4.0 * m_lat) +
                  PB4 * std::sin(6.0 * m_lat) + PB5 * std::sin(8.0 * m_lat) +
                  PB6 * std::sin(10.0 * m_lat) + PB7 * std::sin(12.0 * m_lat) +
                  PB8 * std::sin(14.0 * m_lat) + PB9 * std::sin(16.0 * m_lat);

  const auto PSo = PB1 * m_plane_lat + PB2 * std::sin(2.0 * m_plane_lat) +
                   PB3 * std::sin(4.0 * m_plane_lat) + PB4 * std::sin(6.0 * m_plane_lat) +
                   PB5 * std::sin(8.0 * m_plane_lat) + PB6 * std::sin(10.0 * m_plane_lat) +
                   PB7 * std::sin(12.0 * m_plane_lat) + PB8 * std::sin(14.0 * m_plane_lat) +
                   PB9 * std::sin(16.0 * m_plane_lat);

  const auto PDL = m_lon - m_plane_lon;
  const auto Pt = std::tan(m_lat);
  const auto PW = std::sqrt(1.0 - std::pow(Pe, 2) * std::pow(std::sin(m_lat), 2));
  const auto PN = AW / PW;
  const auto Pnn = std::sqrt(std::pow(Pet, 2) * std::pow(std::cos(m_lat), 2));

  m_y = ((PS - PSo) + (1.0 / 2.0) * PN * std::pow(std::cos(m_lat), 2.0) * Pt * std::pow(PDL, 2.0) +
         (1.0 / 24.0) * PN * std::pow(std::cos(m_lat), 4) * Pt *
           (5.0 - std::pow(Pt, 2) + 9.0 * std::pow(Pnn, 2) + 4.0 * std::pow(Pnn, 4)) *
           std::pow(PDL, 4) -
         (1.0 / 720.0) * PN * std::pow(std::cos(m_lat), 6) * Pt *
           (-61.0 + 58.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 270.0 * std::pow(Pnn, 2) +
            330.0 * std::pow(Pt, 2) * std::pow(Pnn, 2)) *
           std::pow(PDL, 6) -
         (1.0 / 40320.0) * PN * std::pow(std::cos(m_lat), 8) * Pt *
           (-1385.0 + 3111 * std::pow(Pt, 2) - 543 * std::pow(Pt, 4) + std::pow(Pt, 6)) *
           std::pow(PDL, 8)) *
        Pmo;

  m_x = (PN * std::cos(m_lat) * PDL -
         1.0 / 6.0 * PN * std::pow(std::cos(m_lat), 3) * (-1 + std::pow(Pt, 2) - std::pow(Pnn, 2)) *
           std::pow(PDL, 3) -
         1.0 / 120.0 * PN * std::pow(std::cos(m_lat), 5) *
           (-5.0 + 18.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 14.0 * std::pow(Pnn, 2) +
            58.0 * std::pow(Pt, 2) * std::pow(Pnn, 2)) *
           std::pow(PDL, 5) -
         1.0 / 5040.0 * PN * std::pow(std::cos(m_lat), 7) *
           (-61.0 + 479.0 * std::pow(Pt, 2) - 179.0 * std::pow(Pt, 4) + std::pow(Pt, 6)) *
           std::pow(PDL, 7)) *
        Pmo;

  m_z = m_h;
}
////////////////////////////////////////////////////////////////////////////

bool GeoPosConv::check_plane_setting() const
{
  return !(m_plane_lon == 0 && m_plane_lat == 0);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace usv