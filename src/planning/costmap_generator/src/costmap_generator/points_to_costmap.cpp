//
// Created by liuhao on 2021/12/21.
//
/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include <costmap_generator/points_to_costmap.hpp>
#include <utility>

namespace PointsToCoastMap
{
///////////////////////////////////////////////////////////////////////////////////////
grid_map::Matrix PointsToCoastMap::makeCostmapFromSensorPoints(
  double maximum_height_thres,
  double minimum_height_thres,
  double grid_min_value,
  double grid_max_value,
  const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name,
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points)
{
  initGridmapParam(gridmap);
  auto grid_vec = assignPoints2GridCell(std::move(in_sensor_points));
  auto costmap = calculateCostmap(
    maximum_height_thres,
    minimum_height_thres,
    grid_min_value,
    grid_max_value,
    gridmap,
    gridmap_layer_name,
    grid_vec);
  return costmap;
}

///////////////////////////////////////////////////////////////////////////////////////
void PointsToCoastMap::initGridmapParam(const grid_map::GridMap & gridmap)
{
  grid_length_x_ = gridmap.getLength().x();
  grid_length_y_ = gridmap.getLength().y();
  grid_resolution_ = gridmap.getResolution();
  grid_position_x_ = gridmap.getPosition().x();
  grid_position_y_ = gridmap.getPosition().y();
}

///////////////////////////////////////////////////////////////////////////////////////
bool PointsToCoastMap::isValidInx(const grid_map::Index & grid_inx) const
{
  bool is_valid = false;
  int x_grid_inx = grid_inx.x();
  int y_grid_inx = grid_inx.y();
  if (
    x_grid_inx >= 0 && x_grid_inx < std::ceil(grid_length_x_ / grid_resolution_) &&
    y_grid_inx >= 0 && y_grid_inx < std::ceil(grid_length_y_ / grid_resolution_)) {
    is_valid = true;
  }
  return is_valid;
}

///////////////////////////////////////////////////////////////////////////////////////
grid_map::Index PointsToCoastMap::fetchGridIndexFromPoint(const pcl::PointXYZ & point) const
{
  // calculate out_grid_map position
  const double origin_x_offset = grid_length_x_ / 2.0 - grid_position_x_;
  const double origin_y_offset = grid_length_y_ / 2.0 - grid_position_y_;
  // coordinate conversion for making index. Set bottom left to the origin of coordinate (0, 0) in
  // gridmap area
  const double mapped_x = (grid_length_x_ - origin_x_offset - point.x) / grid_resolution_;
  const double mapped_y = (grid_length_y_ - origin_y_offset - point.y) / grid_resolution_;

  const int mapped_x_inx = std::ceil(mapped_x);
  const int mapped_y_inx = std::ceil(mapped_y);
  grid_map::Index index{mapped_x_inx, mapped_y_inx};
  return index;
}

///////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<std::vector<double>>> PointsToCoastMap::assignPoints2GridCell(
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points) const
{
  const size_t y_cell_size = std::ceil(grid_length_y_ * (1 / grid_resolution_));
  const size_t x_cell_size = std::ceil(grid_length_x_ * (1 / grid_resolution_));
  std::vector<double> z_vec;
  std::vector<std::vector<double>> vec_y_z(y_cell_size, z_vec);
  std::vector<std::vector<std::vector<double>>> vec_x_y_z(x_cell_size, vec_y_z);

  for (const auto & point : *in_sensor_points) {
    grid_map::Index grid_ind = fetchGridIndexFromPoint(point);
    if (isValidInx(grid_ind)) {
      vec_x_y_z[grid_ind.x()][grid_ind.y()].push_back(point.z);
    }
  }
  return vec_x_y_z;
}

///////////////////////////////////////////////////////////////////////////////////////
grid_map::Matrix PointsToCoastMap::calculateCostmap(
  double maximum_height_thres,
  double minimum_height_thres,
  double grid_min_value,
  double grid_max_value,
  const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name,
  std::vector<std::vector<std::vector<double>>> grid_vec) const
{
  grid_map::Matrix gridmap_data = gridmap[gridmap_layer_name];
  for (size_t x_inx = 0; x_inx < grid_vec.size(); x_inx++) {
    for (size_t y_inx = 0; y_inx < grid_vec[0].size(); y_inx++) {
      if (grid_vec[x_inx][y_inx].empty()) {
        gridmap_data(x_inx, y_inx) = grid_min_value;
        continue;
      }
      for (const auto & z : grid_vec[x_inx][y_inx]) {
        if (z > maximum_height_thres || z < minimum_height_thres) {
          continue;
        }
        gridmap_data(x_inx, y_inx) = grid_max_value;
        break;
      }
    }
  }
  return gridmap_data;
}


}  // namespace PointsToCoastMap