//
// Created by liuhao on 22-4-20.
//

#include <gtest/gtest.h>

#include <gnss/geo_pos_conv.hpp>

using usv::drivers::gnss::GeoPosConv;

TEST(Checks, LlhConv)
{
  auto test = GeoPosConv{};
  const auto plane_lon = 0.;
  const auto plane_lat = 0.;
  test.set_plane_degree(plane_lon, plane_lat);
  test.set_llh_degree(plane_lon + 1e-3, plane_lat, 0.);
  std::cout << test.x() << ' ' << test.y() << '\n';
}

TEST(test, test)
{
  EXPECT_EQ(1 + 1, 2);
}
