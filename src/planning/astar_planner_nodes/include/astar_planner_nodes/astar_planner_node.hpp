#ifndef ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <astar_planner_nodes/visibility_control.hpp>
#include <astar_search/astar_search.hpp>
#include <motion_common/motion_common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <usv_msgs/action/plan_trajectory.hpp>
#include <usv_msgs/action/planner_costmap.hpp>
#include <usv_msgs/msg/trajectory.hpp>


namespace usv
{
namespace planning
{
namespace astar_planner
{

enum class AstarPlannerState { IDLE, PLANNING };


struct NodeParam
{
  double waypoints_velocity;  // constant velocity on planned waypoints [m/s]
};

/// \class AstarPlannerNode
/// \brief Creates Hybrid A* planning algorithm node
class AstarPlannerNode : public rclcpp::Node
{
public:
  /// \brief Default constructor for AstarPlannerNode class
  /// \param [in] node_options A rclcpp::NodeOptions object passed on to rclcpp::Node
  explicit AstarPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  using PlanTrajectoryAction = usv_msgs::action::PlanTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanTrajectoryAction>;
  using PlannerCostmapAction = usv_msgs::action::PlannerCostmap;
  using PlannerCostmapGoalHandle = rclcpp_action::ClientGoalHandle<PlannerCostmapAction>;

  // ros communication
  rclcpp_action::Client<PlannerCostmapAction>::SharedPtr map_client_;
  rclcpp_action::Server<PlanTrajectoryAction>::SharedPtr plan_trajectory_srv_;
  rclcpp::Publisher<usv_msgs::msg::Trajectory>::SharedPtr trajectory_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_trajectory_debug_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_{};
  astar_search::AstarParam astar_param_{};
  AstarPlannerState state_{};

  // variables
  std::unique_ptr<astar_search::AstarSearch> astar_;
  std::shared_ptr<GoalHandle> planning_goal_handle_{nullptr};
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  usv_msgs::msg::Trajectory trajectory_;
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;

  // callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PlanTrajectoryAction::Goal>);

  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle);

  void goalResponseCallback(std::shared_future<PlannerCostmapGoalHandle::SharedPtr> future);

  void feedbackCallback(
    PlannerCostmapGoalHandle::SharedPtr,
    const std::shared_ptr<const PlannerCostmapAction::Feedback> &)
  {
  }

  void resultCallback(const PlannerCostmapGoalHandle::WrappedResult & result);

  // functions
  void reset();

  bool isPlanning() const;

  void startPlanning();

  void stopPlanning();

  bool planTrajectory();

  geometry_msgs::msg::TransformStamped getTransform(
    const std::string & from, const std::string & to);

  void visualizeTrajectory();
};

}  // namespace astar_planner
}  // namespace planning
}  // namespace usv

#endif  // ASTAR_PLANNER__ASTAR_PLANNER_HPP_
