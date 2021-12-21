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

#ifndef COSTMAP_GENERATOR_POINTS_TO_COSTMAP_HPP
#define COSTMAP_GENERATOR_POINTS_TO_COSTMAP_HPP

#include <costmap_generator/visibility_control.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace PointsToCoastMap
{

class COSTMAP_GENERATOR_PUBLIC PointsToCoastMap
{
public:
  PointsToCoastMap() = default;
  ~PointsToCoastMap() = default;

  /// \brief calculate cost from sensor points
  /// \param[in] maximum_height_thres: Maximum height threshold for pointcloud data
  /// \param[in] minimum_height_thres: Minimum height threshold for pointcloud data
  /// \param[in] grid_min_value: Minimum cost for costmap
  /// \param[in] grid_max_value: Maximum cost fot costmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] gridmap_layer_name: gridmap layer name for gridmap
  /// \param[in] in_sensor_points: subscribed pointcloud
  /// \param[out] calculated cost in grid_map::Matrix format
  grid_map::Matrix makeCostmapFromSensorPoints(
    double maximum_height_thres,
    double minimum_height_thres,
    double grid_min_value,
    double grid_max_value,
    const grid_map::GridMap & gridmap,
    const std::string & gridmap_layer_name,
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points);

private:
  /// \brief initialize gridmap parameters
  /// \param[in] gridmap: gridmap object to be initialized
  COSTMAP_GENERATOR_LOCAL void initGridmapParam(const grid_map::GridMap & gridmap);

  /// \brief check if index is valid in the gridmap
  /// \param[in] grid_inx: grid index corresponding with one of pointcloud
  /// \param[out] bool: true if index is valid
  COSTMAP_GENERATOR_LOCAL bool isValidInx(const grid_map::Index & grid_inx) const;

  /// \brief Get index from one of pointcloud
  /// \param[in] point: one of subscribed pointcloud
  /// \param[out] index in gridmap
  COSTMAP_GENERATOR_LOCAL grid_map::Index fetchGridIndexFromPoint(
    const pcl::PointXYZ & point) const;

  /// \brief Assign pointcloud to appropriate cell in gridmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] in_sensor_points: subscribed pointcloud
  /// \param[out] grid-x-length x grid-y-length size grid stuffed with point's height in
  /// corresponding grid cell
  COSTMAP_GENERATOR_LOCAL std::vector<std::vector<std::vector<double>>> assignPoints2GridCell(
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points) const;

  /// \brief calculate costmap from subscribed pointcloud
  /// \param[in] maximum_height_thres: Maximum height threshold for pointcloud data
  /// \param[in] minimum_height_thres: Minimum height threshold for pointcloud data
  /// \param[in] grid_min_value: Minimum cost for costmap
  /// \param[in] grid_max_value: Maximum cost fot costmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] gridmap_layer_name: gridmap layer name for gridmap
  /// \param[in] grid_vec: grid-x-length x grid-y-length size grid stuffed with point's height in
  /// corresponding grid cell \param[out] calculated costmap in grid_map::Matrix format
  COSTMAP_GENERATOR_LOCAL grid_map::Matrix calculateCostmap(
    double maximum_height_thres,
    double minimum_height_thres,
    double grid_min_value,
    double grid_max_value,
    const grid_map::GridMap & gridmap,
    const std::string & gridmap_layer_name,
    std::vector<std::vector<std::vector<double>>> grid_vec) const;

  double grid_length_x_;
  double grid_length_y_;
  double grid_resolution_;
  double grid_position_x_;
  double grid_position_y_;
};
}  // namespace PointsToCoastMap

#endif  // COSTMAP_GENERATOR_POINTS_TO_COSTMAP_HPP
