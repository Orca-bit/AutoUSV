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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

#include "costmap_generator/costmap_generator.hpp"

// TODO TEST
using usv::planning::costmap_generator::CostmapGenerator;
using usv::planning::costmap_generator::CostmapGeneratorParams;
using usv::planning::costmap_generator::LayerName;

CostmapGeneratorParams generateExampleParameters()
{
  auto params = CostmapGeneratorParams();

  params.grid_min_value = 0.0;
  params.grid_max_value = 1.0;
  params.grid_resolution = 0.2;
  params.grid_length_x = 70.0;
  params.grid_length_y = 70.0;
  params.grid_position_x = 0.0;
  params.grid_position_y = 0.0;
  params.costmap_frame = "map";

  return params;
}

TEST(CostmapGeneratorTest, GenerateValidCostmap)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters();
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  auto cell_value_treshold =
    static_cast<float>((costmap_param.grid_min_value + costmap_param.grid_max_value) / 2.0);

  // treshold for cells not specifically inside lanelet
  auto mismatch_treshold = 10;

  auto number_of_mismatches = countCostmapToLaneletMismatch(
    costmap,
    lanelet_map_ptr,
    LayerName::COMBINED,
    costmap_param.grid_resolution,
    cell_value_treshold);

  EXPECT_LT(number_of_mismatches, mismatch_treshold);
}

TEST(CostmapGeneratorTest, GenerateBoundedCostmap)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters();
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  EXPECT_LT(costmap.getLength().x(), costmap_param.grid_length_x);
  EXPECT_LT(costmap.getLength().y(), costmap_param.grid_length_y);
}

TEST(CostmapGeneratorTest, GenerateCostmapWithoutBounding)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters(false);
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  EXPECT_DOUBLE_EQ(costmap.getResolution(), costmap_param.grid_resolution);
  EXPECT_DOUBLE_EQ(costmap.getLength().x(), costmap_param.grid_length_x);
  EXPECT_DOUBLE_EQ(costmap.getLength().y(), costmap_param.grid_length_y);
}

TEST(CostmapGeneratorTest, GenerateCostmapWithoutWayareasExpectOnlyObstacles)
{
  // initialize costmap generator
  auto costmap_param = generateExampleParameters(false, false);
  auto costmap_generator = std::make_unique<CostmapGenerator>(costmap_param);

  auto lanelet_map_ptr = createLanelet(costmap_param.grid_resolution);

  // grid is placed in the middle of map frame
  grid_map::Position vehicle_to_grid_position(0.0, 0.0);
  // map frame = costmap frame
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;

  auto costmap = costmap_generator->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  const grid_map::Matrix & map = costmap[LayerName::COMBINED];
  for (grid_map::GridMapIterator iterator(costmap); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const auto cell_value = static_cast<double>(map(index(0), index(1)));
    if (cell_value != costmap_param.grid_max_value) {
      ASSERT_TRUE(false);
    }
  }
}

TEST(CostmapGeneratorTest, InitializeCostmapWithZeroResolution)
{
  // initialize costmap generator
  auto costmap_params = generateExampleParameters();
  costmap_params.grid_resolution = 0.0;
  EXPECT_THROW(std::make_unique<CostmapGenerator>(costmap_params), std::invalid_argument);
}

TEST(CostmapGeneratorTest, InitializeCostmapWithZeroXLength)
{
  // initialize costmap generator
  auto costmap_params = generateExampleParameters();
  costmap_params.grid_length_x = 0.0;
  EXPECT_THROW(std::make_unique<CostmapGenerator>(costmap_params), std::invalid_argument);
}

TEST(CostmapGeneratorTest, InitializeCostmapWithZeroYLength)
{
  // initialize costmap generator
  auto costmap_params = generateExampleParameters();
  costmap_params.grid_length_y = 0.0;
  EXPECT_THROW(std::make_unique<CostmapGenerator>(costmap_params), std::invalid_argument);
}
