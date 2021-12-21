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
 *  * Redistributions of source code must retain the above copyright notice, private_node
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    private_node list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    private_node software without specific prior written permission.
 *
 *  private_node SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF private_node SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "costmap_generator/costmap_generator.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <utility>

namespace usv
{
namespace planning
{
namespace costmap_generator
{
CostmapGenerator::CostmapGenerator(CostmapGeneratorParams generator_params)
: params_(std::move(generator_params))
{
  auto throw_if_non_positive = [](double number, const std::string & name) {
    if (number <= 0.0) {
      throw std::invalid_argument(name + " = " + std::to_string(number) + " should be positive.");
    }
  };

  throw_if_non_positive(params_.grid_resolution, "params_.grid_resolution");
  throw_if_non_positive(params_.grid_length_x, "params_.grid_length_x");
  throw_if_non_positive(params_.grid_length_y, "params_.grid_length_y");

  costmap_.setFrameId(params_.costmap_frame);
  costmap_.setGeometry(
    grid_map::Length(params_.grid_length_x, params_.grid_length_y),
    params_.grid_resolution,
    grid_map::Position(params_.grid_position_x, params_.grid_position_y));

  costmap_.add(LayerName::LIDAR, params_.grid_max_value);
  costmap_.add(LayerName::COMBINED, params_.grid_min_value);
}

grid_map::GridMap CostmapGenerator::generateCostmap(
  const grid_map::Position & vehicle_to_grid_position)
{
  // Move grid map with data to robot's center position
  costmap_.setPosition(vehicle_to_grid_position);

  costmap_[LayerName::LIDAR] = generateSensorPointsCostmap();

  costmap_[LayerName::COMBINED] = generateCombinedCostmap();

  auto result = costmap_;

  return result;
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap() const
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::COMBINED].setConstant(static_cast<float>(params_.grid_min_value));

  combined_costmap[LayerName::COMBINED] =
    combined_costmap[LayerName::COMBINED].cwiseMax(combined_costmap[LayerName::LIDAR]);

  return combined_costmap[LayerName::COMBINED];
}

grid_map::Matrix CostmapGenerator::generateSensorPointsCostmap() const
{
  if (point_cloud_ == nullptr) {
    return {};
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::fromROSMsg(*point_cloud_, *in_sensor_points);
  auto sensor_points_costmap = points_to_costmap_->makeCostmapFromSensorPoints(
    params_.max_lidar_height_thres,
    params_.min_lidar_height_thres,
    params_.grid_min_value,
    params_.grid_max_value,
    costmap_,
    LayerName::LIDAR,
    std::move(in_sensor_points));
  return sensor_points_costmap;
}

void CostmapGenerator::set_point_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr points)
{
  point_cloud_ = std::move(points);
}

}  // namespace costmap_generator
}  // namespace planning
}  // namespace usv
