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

#ifndef COSTMAP_GENERATOR_NODES__COSTMAP_GENERATOR_NODE_HPP_
#define COSTMAP_GENERATOR_NODES__COSTMAP_GENERATOR_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include <costmap_generator/costmap_generator.hpp>
#include <costmap_generator_nodes/visibility_control.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <usv_msgs/action/planner_costmap.hpp>


namespace usv
{
namespace planning
{
namespace costmap_generator
{
using PlannerCostmapAction = usv_msgs::action::PlannerCostmap;

enum class CostmapGeneratorNodeState { IDLE, GENERATING };

/// \class CostmapGeneratorNode
/// \brief Creates costmap generation algorithm node
class CostmapGeneratorNode : public rclcpp::Node
{
public:
  /// \brief Default constructor for CostmapGeneratorNode class
  /// \param [in] node_options A rclcpp::NodeOptions object passed on to rclcpp::Node
  explicit CostmapGeneratorNode(const rclcpp::NodeOptions & node_options);

private:
  std::string vehicle_frame_;

  CostmapGeneratorParams costmap_params_;
  std::unique_ptr<CostmapGenerator> costmap_generator_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_occupancy_grid_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  rclcpp_action::Server<PlannerCostmapAction>::SharedPtr costmap_action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PlannerCostmapAction>> goal_handle_{nullptr};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // action
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PlannerCostmapAction::Goal>);
  rclcpp_action::CancelResponse handleCancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PlannerCostmapAction>>);
  void handleAccepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<PlannerCostmapAction>>);

  grid_map::Position getCostmapToVehicleTranslation();

  void mapResponse();

  CostmapGeneratorNodeState state_;
  bool isIdle() const { return state_ == CostmapGeneratorNodeState::IDLE; }
  bool isGenerating() const { return state_ == CostmapGeneratorNodeState::GENERATING; }
  void setIdleState() { state_ = CostmapGeneratorNodeState::IDLE; }
  void setGeneratingState() { state_ = CostmapGeneratorNodeState::GENERATING; }
};

}  // namespace costmap_generator
}  // namespace planning
}  // namespace usv

#endif  // COSTMAP_GENERATOR_NODES__COSTMAP_GENERATOR_NODE_HPP_
