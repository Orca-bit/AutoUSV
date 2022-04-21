//
// Created by liuhao on 22-4-19.
//

#ifndef GNSS_GEO_POS_CONV_HPP
#define GNSS_GEO_POS_CONV_HPP

#include "gnss/visibility_control.hpp"

namespace usv
{
namespace drivers
{
namespace gnss
{
class GNSS_INTERFACE_PUBLIC GeoPosConv
{
public:
  GeoPosConv() = default;
  ~GeoPosConv() = default;
  /// get current x [m]
  double x() const;
  /// get current y [m]
  double y() const;
  /// get current z [m]
  double z() const;
  /// set plane longitude and latitude [degrees]
  void set_plane_degree(double lon, double lat);
  /// set plane longitude and latitude [radians]
  void set_plane_radian(double lon, double lat);
  /// set longitude [degrees] latitude [degrees] and height [m]
  void set_llh_degree(double lon, double lat, double h);
  /// set longitude [radians] latitude [radians] and height [m]
  void set_llh_radian(double lon, double lat, double h);
  /// check the plane longitude and latitude
  /// return false if both are zero
  bool check_plane_setting() const;


private:
  /// do the actual conversion
  GNSS_INTERFACE_LOCAL void conv_llh2xyz();
  /// x [m] in x-y-z coordinate
  double m_x{0.};
  /// y [m] in x-y-z coordinate
  double m_y{0.};
  /// z [m] in x-y-z coordinate
  double m_z{0.};
  /// longitude [radians]
  double m_lon{0.};
  /// latitude [radians]
  double m_lat{0.};
  /// height [m]
  double m_h{0.};
  /// plane longitude [radians]
  double m_plane_lon{0.};
  /// plane latitude [radians]
  double m_plane_lat{0.};
};
}  // namespace gnss
}  // namespace drivers
}  // namespace usv
#endif  // GNSS_GEO_POS_CONV_HPP
