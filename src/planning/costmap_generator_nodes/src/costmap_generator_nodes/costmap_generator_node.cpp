// Copyright (c) 2021 AutoUSV
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

#include "costmap_generator_nodes/costmap_generator_node.hpp"

#include <memory>
#include <string>
#include <utility>

#include "tf2/utils.h"
#include <time_utils/time_utils.hpp>

nav_msgs::msg::OccupancyGrid createOccupancyGrid(
  const grid_map::GridMap & costmap,
  const std::string & layer_name,
  usv::planning::costmap_generator::CostmapGeneratorParams & costmap_params)
{
  nav_msgs::msg::OccupancyGrid occupancy_grid;

  // Convert to OccupancyGrid
  grid_map::GridMapRosConverter::toOccupancyGrid(
    costmap,
    layer_name,
    static_cast<float>(costmap_params.grid_min_value),
    static_cast<float>(costmap_params.grid_max_value),
    occupancy_grid);

  // Set header
  std_msgs::msg::Header header;
  header.frame_id = costmap_params.costmap_frame;
  {
    using time_utils::to_message;
    header.stamp = to_message(std::chrono::system_clock::now());
  }
  occupancy_grid.header = header;

  return occupancy_grid;
}

namespace usv
{
namespace planning
{
namespace costmap_generator
{
CostmapGeneratorNode::CostmapGeneratorNode(const rclcpp::NodeOptions & node_options)
: Node("costmap_generator_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Node Parameters
  vehicle_frame_ = this->declare_parameter<std::string>("vehicle_frame", "base_link");

  // Costmap Parameters
  costmap_params_.grid_min_value = this->declare_parameter<double>("grid_min_value", 0.0);
  costmap_params_.grid_max_value = this->declare_parameter<double>("grid_max_value", 1.0);
  costmap_params_.min_lidar_height_thres =
    this->declare_parameter<double>("min_lidar_height_thres", -1.0);
  costmap_params_.max_lidar_height_thres =
    this->declare_parameter<double>("max_lidar_height_thres", 1.0);
  costmap_params_.grid_resolution = this->declare_parameter<double>("grid_resolution", 0.2);
  costmap_params_.grid_length_x = this->declare_parameter<double>("grid_length_x", 50);
  costmap_params_.grid_length_y = this->declare_parameter<double>("grid_length_y", 50);
  costmap_params_.grid_position_x = this->declare_parameter<double>("grid_position_x", 0);
  costmap_params_.grid_position_y = this->declare_parameter<double>("grid_position_y", 0);
  costmap_params_.costmap_frame = this->declare_parameter<std::string>("costmap_frame", "map");

  // Initialize costmap generator
  costmap_generator_ = std::make_unique<CostmapGenerator>(costmap_params_);

  // Wait for first tf
  // We want to do this before creating subscriptions
  while (rclcpp::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Waiting for initial pose...");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // setup point cloud subscription
  {
    auto cb = [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      this->costmap_generator_->set_point_cloud(std::move(msg));
    };
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_no_ground", rclcpp::QoS{10U}, cb);
  }

  // Publishers
  debug_occupancy_grid_publisher_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/occupancy_grid", 1);

  // Action server
  costmap_action_server_ = rclcpp_action::create_server<PlannerCostmapAction>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "generate_costmap",
    [this](auto uuid, auto goal) { return this->handleGoal(uuid, goal); },
    [this](auto goal_handle) { return this->handleCancel(goal_handle); },
    [this](auto goal_handle) { return this->handleAccepted(goal_handle); });
}

rclcpp_action::GoalResponse CostmapGeneratorNode::handleGoal(
  const rclcpp_action::GoalUUID &, const std::shared_ptr<const PlannerCostmapAction::Goal>)
{
  if (!isIdle()) {
    RCLCPP_WARN(get_logger(), "Costmap generator is not in idle. Rejecting new request.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received new costmap request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CostmapGeneratorNode::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlannerCostmapAction>>)
{
  RCLCPP_WARN(get_logger(), "Received action cancellation, rejecting.");
  return rclcpp_action::CancelResponse::REJECT;
}

void CostmapGeneratorNode::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlannerCostmapAction>>)
{
  setGeneratingState();
  mapResponse();
}

grid_map::Position CostmapGeneratorNode::getCostmapToVehicleTranslation()
{
  // Get current pose
  geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
    costmap_params_.costmap_frame,
    vehicle_frame_,
    rclcpp::Time(0),
    rclcpp::Duration::from_seconds(1.0));

  return {tf.transform.translation.x, tf.transform.translation.y};
}

void CostmapGeneratorNode::mapResponse()
{
  // Find translation from grid map to robots center position
  grid_map::Position vehicle_to_grid_position;
  try {
    vehicle_to_grid_position = getCostmapToVehicleTranslation();
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Setting costmap position failure: %s", ex.what());
    return;
  }

  // Generate costmap
  auto costmap = costmap_generator_->generateCostmap(vehicle_to_grid_position);

  // Create result
  auto result = std::make_shared<PlannerCostmapAction::Result>();
  auto out_occupancy_grid = createOccupancyGrid(costmap, LayerName::COMBINED, costmap_params_);
  result->costmap = out_occupancy_grid;

  // Publish visualizations
  debug_occupancy_grid_publisher_->publish(out_occupancy_grid);

  goal_handle_->succeed(result);

  RCLCPP_INFO(get_logger(), "Costmap generation succeeded.");

  setIdleState();
}

}  // namespace costmap_generator
}  // namespace planning
}  // namespace usv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usv::planning::costmap_generator::CostmapGeneratorNode)
