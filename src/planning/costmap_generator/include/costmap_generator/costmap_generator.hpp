// Copyright (c) 2021, AutoUSV
//
// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

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

#ifndef COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <costmap_generator/points_to_costmap.hpp>
#include <costmap_generator/visibility_control.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace usv
{
namespace planning
{
namespace costmap_generator
{
/// \brief Struct holding costmap layer names
struct COSTMAP_GENERATOR_PUBLIC LayerName
{
  /// Name of layer only with lidar sensor info
  static constexpr const char * LIDAR = "lidar";
  /// Name of layer only with detected objects info
  static constexpr const char * OBJECTS = "objects";
  /// Name of layer which has all information applied
  static constexpr const char * COMBINED = "combined";
};

/// \brief Parameters for CostmapGenerator class
struct COSTMAP_GENERATOR_PUBLIC CostmapGeneratorParams
{
  double grid_min_value;          ///< Minimal costmap grid value (free space)
  double grid_max_value;          ///< Maximal costmap grid value (obstacle)
  double grid_resolution;         ///< Costmap resolution
  double grid_length_x;           ///< Costmap x direction size
  double grid_length_y;           ///< Costmap y direction size
  double grid_position_x;         ///< X position of costmap in its frame
  double grid_position_y;         ///< Y position of costmap in its frame
  double max_lidar_height_thres;  ///< filter on points height
  double min_lidar_height_thres;  ///< filter on points height
  std::string costmap_frame;      ///< Costmap frame name
};

/// \class CostmapGenerator
/// \brief Costmap generation algorithm regarding point clouds data
class COSTMAP_GENERATOR_PUBLIC CostmapGenerator
{
public:
  /// \brief Initialize gridmap parameters based on ros param
  /// \param [in] generator_params Costmap generation algorithm configuration
  explicit CostmapGenerator(CostmapGeneratorParams generator_params);

  /// \brief Generate costmap
  /// \details This is main function which is capable of creating
  ///          GridMap, basing on lanelet2 information and
  ///          transforms between map, costmap, and vehicle
  ///          positions. It is possible to decide if the function
  ///          should truncate the final costmap or apply driveable
  ///          areas by setting proper configuration parameters
  /// \param [in] vehicle_to_grid_position Translation between costmap and vehicle frame used to
  ///                                     align costmap and vehicle center
  /// \return Generated costmap
  grid_map::GridMap generateCostmap(const grid_map::Position & vehicle_to_grid_position);

  void set_point_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr points);

private:
  grid_map::GridMap costmap_{};
  CostmapGeneratorParams params_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_{nullptr};
  std::unique_ptr<PointsToCoastMap::PointsToCoastMap> points_to_costmap_{
    std::make_unique<PointsToCoastMap::PointsToCoastMap>()};

  /// \brief calculate cost from lidar data
  grid_map::Matrix generateSensorPointsCostmap() const;

  /// \brief Calculate costmap layer costs for final output
  /// \return Costmap layer
  grid_map::Matrix generateCombinedCostmap() const;
};

}  // namespace costmap_generator
}  // namespace planning
}  // namespace usv

#endif  // COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
