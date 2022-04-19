#include "astar_planner_nodes/astar_planner_node.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>


namespace usv
{
namespace planning
{
namespace astar_planner
{

/// request cost-map
using usv_msgs::action::PlannerCostmap;
/// publish trajectory
using usv_msgs::msg::Trajectory;


/// do pose transform
geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

astar_search::AstarWaypoints adjustWaypointsSize(
  const astar_search::AstarWaypoints & astar_waypoints)
{
  auto max_length = Trajectory::CAPACITY;
  auto input_length = astar_waypoints.waypoints.size();

  if (input_length > max_length) {
    astar_search::AstarWaypoints resized_vector;
    resized_vector.header = astar_waypoints.header;

    // input_length subtraction to handle max_length multiplicity
    auto elements_to_skip_per_step = static_cast<int64_t>(std::floor(input_length / max_length));
    // must be +1 to actually skip an element
    // for example: input_length = 70, max_length = 50, elements_to_skip_per_step = 1, which
    // actually not to skip
    auto stride = elements_to_skip_per_step + 1;

    auto waypoints_iter = astar_waypoints.waypoints.cbegin();
    // total number of points need to skip
    auto points_to_skip = static_cast<int64_t>(input_length - max_length);
    int64_t skipped_points_count = 0;
    // subtract by elements_to_skip_per_step to prevent from skipping too many points
    while (skipped_points_count < points_to_skip) {
      resized_vector.waypoints.push_back(*waypoints_iter);
      waypoints_iter += std::min(stride, points_to_skip - skipped_points_count + 1);
      skipped_points_count += elements_to_skip_per_step;
    }

    // complete skips, if there are some points rest
    // copy rest of waypoints
    resized_vector.waypoints.insert(
      resized_vector.waypoints.end(), waypoints_iter, astar_waypoints.waypoints.cend());
    return resized_vector;
  }

  // if input size < max size, just return (a copy)
  return astar_waypoints;
}

usv_msgs::msg::Trajectory createTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const astar_search::AstarWaypoints & astar_waypoints,
  const float & velocity)
{
  Trajectory trajectory;
  trajectory.header = astar_waypoints.header;

  // awp: astar waypoint
  for (const auto & awp : astar_waypoints.waypoints) {
    usv_msgs::msg::TrajectoryPoint point;

    point.pose.position.x = awp.pose.pose.position.x;
    point.pose.position.y = awp.pose.pose.position.y;
    // z is not matter
    point.pose.position.z =
      current_pose.pose.position.z;  // height = const, not important for 2D implementation
    point.pose.orientation = awp.pose.pose.orientation;

    // switch sign by forward/backward
    // velocity = const
    point.longitudinal_velocity_mps = (awp.is_back ? -1.0f : 1.0f) * velocity;
    // the transverse velocity and angular velocity are not to set (default value: 0)
    // for controller reference, this does make sense

    trajectory.points.push_back(point);
  }

  return trajectory;
}

AstarPlannerNode::AstarPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("astar_planner", node_options)
{
  using namespace std::literals::chrono_literals;

  auto throw_if_negative = [](int64_t number, const std::string & name) {
    if (number < 0) {
      throw std::runtime_error(name + " = " + std::to_string(number) + " shouldn't be negative.");
    }
  };

  // NodeParam
  {
    node_param_.waypoints_velocity = declare_parameter("waypoints_velocity", 2.0);
  }

  // AstarParam
  {
    // base configs
    astar_param_.use_back = declare_parameter("use_back", true);
    astar_param_.only_behind_solutions = declare_parameter("only_behind_solutions", false);
    astar_param_.time_limit = declare_parameter("time_limit", 5000.0);

    auto vehicle_dimension_margin = declare_parameter("vehicle_dimension_margin", 0.0);
    astar_param_.robot_shape.length =
      declare_parameter("vehicle_length", 0.0) + vehicle_dimension_margin;
    astar_param_.robot_shape.width =
      declare_parameter("vehicle_width", 0.0) + vehicle_dimension_margin;
    astar_param_.robot_shape.cg2back =
      declare_parameter("cg_to_back", 0.0) + (vehicle_dimension_margin / 2.0);
    astar_param_.minimum_turning_radius = declare_parameter("minimum_turning_radius", 2.0);

    astar_param_.maximum_turning_radius = declare_parameter("maximum_turning_radius", 6.0);
    astar_param_.maximum_turning_radius =
      std::max(astar_param_.maximum_turning_radius, astar_param_.minimum_turning_radius);
    auto tr_size = declare_parameter("turning_radius_size", 11);
    throw_if_negative(tr_size, "turning_radius_size");
    /// Number of levels of discretization between minimum and maximum turning radius [-]
    astar_param_.turning_radius_size = static_cast<size_t>(tr_size);

    // search configs
    auto th_size = declare_parameter("theta_size", 48);
    throw_if_negative(th_size, "theta_size");
    /// Number of possible headings, discretized between <0, 2pi> [-]
    astar_param_.theta_size = static_cast<size_t>(th_size);
    // TODO: Check the usage
    astar_param_.reverse_weight = declare_parameter("reverse_weight", 2.00);
    astar_param_.goal_lateral_tolerance = declare_parameter("goal_lateral_tolerance", 0.25);
    astar_param_.goal_longitudinal_tolerance =
      declare_parameter("goal_longitudinal_tolerance", 1.0);
    astar_param_.goal_angular_tolerance = declare_parameter("goal_angular_tolerance", 0.05236);

    // costmap configs
    astar_param_.obstacle_threshold = declare_parameter("obstacle_threshold", 100);
    // TODO: Check the usage
    astar_param_.distance_heuristic_weight = declare_parameter("distance_heuristic_weight", 1.0);
  }

  // Hybrid A* implementation
  {
    astar_ = std::make_unique<astar_search::AstarSearch>(astar_param_);
  }

  // debug Publisher
  // the actual path is delivered by client-server protocol
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_debug_pub_ = create_publisher<usv_msgs::msg::Trajectory>("~/debug/trajectory", qos);
    pose_array_trajectory_debug_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("~/debug/trajectory_pose_array", qos);
  }

  // Action client
  {
    map_client_ = rclcpp_action::create_client<PlannerCostmapAction>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "generate_costmap");

    while (!map_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for map server.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for costmap generator action service...");
    }
  }

  // Action service
  {
    plan_trajectory_srv_ = rclcpp_action::create_server<PlanTrajectoryAction>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "astar_plan_trajectory",
      [this](auto uuid, auto goal) { return this->handleGoal(uuid, goal); },
      [this](auto goal_handle) { return this->handleCancel(goal_handle); },
      [this](auto goal_handle) { return this->handleAccepted(goal_handle); });
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  }
}

rclcpp_action::GoalResponse AstarPlannerNode::handleGoal(
  const rclcpp_action::GoalUUID &, const std::shared_ptr<const PlanTrajectoryAction::Goal>)
{
  if (isPlanning()) {
    RCLCPP_WARN(get_logger(), "Planner is already planning. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received new goal.");
  startPlanning();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AstarPlannerNode::handleCancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancelling trajectory planning.");
  stopPlanning();
  auto result = std::make_shared<PlanTrajectoryAction::Result>();
  result->result = PlanTrajectoryAction::Result::FAIL;
  goal_handle->canceled(result);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void AstarPlannerNode::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Astar Planning.");

  planning_goal_handle_ = goal_handle;

  // acquire start and goal position from action request
  start_pose_ = goal_handle->get_goal()->start_pose;
  goal_pose_ = goal_handle->get_goal()->goal_pose;

  // request costmap and plan trajectory
  auto action_goal = PlannerCostmapAction::Goal();
  action_goal.start_pose = start_pose_;
  action_goal.goal_pose = goal_pose_;

  auto send_goal_options = rclcpp_action::Client<PlannerCostmapAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](auto && PH1) {
    goalResponseCallback(std::forward<decltype(PH1)>(PH1));
  };
  send_goal_options.feedback_callback = [this](auto && PH1, auto && PH2) {
    feedbackCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  send_goal_options.result_callback = [this](auto && PH1) {
    resultCallback(std::forward<decltype(PH1)>(PH1));
  };

  map_client_->async_send_goal(action_goal, send_goal_options);
  RCLCPP_INFO(get_logger(), "Costmap generator action goal sent.");
}

void AstarPlannerNode::goalResponseCallback(
  std::shared_future<PlannerCostmapGoalHandle::SharedPtr> future)
{
  if (!future.get()) {
    RCLCPP_ERROR(get_logger(), "Costmap generator goal was rejected by server.");
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::FAIL;
    planning_goal_handle_->abort(result);
    stopPlanning();
    return;
  }

  RCLCPP_INFO(get_logger(), "Costmap generator action goal accepted.");
}

void AstarPlannerNode::resultCallback(
  const PlannerCostmapGoalHandle::WrappedResult & costmap_result)
{
  auto planning_result = std::make_shared<PlanTrajectoryAction::Result>();

  RCLCPP_INFO(get_logger(), "Costmap received, planning started.");

  occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap_result.result->costmap);

  bool success = planTrajectory();

  if (success) {
    visualizeTrajectory();
    planning_result->result = PlanTrajectoryAction::Result::SUCCESS;
    planning_result->trajectory = trajectory_;
    planning_goal_handle_->succeed(planning_result);
  } else {
    planning_result->result = PlanTrajectoryAction::Result::FAIL;
    planning_goal_handle_->abort(planning_result);
  }

  stopPlanning();
}

bool AstarPlannerNode::planTrajectory()
{
  // reset inner state
  reset();

  // Supply latest costmap to planning algorithm
  astar_->setOccupancyGrid(*occupancy_grid_.get());

  // Calculate poses in costmap frame
  const auto start_pose_in_costmap_frame = transformPose(
    start_pose_.pose, getTransform(occupancy_grid_->header.frame_id, start_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = transformPose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute astar search
  const rclcpp::Time start = get_clock()->now();
  auto search_status = astar_->makePlan(start_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Astar planning took %f [s]", (end - start).seconds());

  if (astar_search::isSuccess(search_status)) {
    RCLCPP_INFO(get_logger(), "Plan found.");
    auto waypoints = adjustWaypointsSize(astar_->getWaypoints());
    trajectory_ =
      createTrajectory(start_pose_, waypoints, static_cast<float>(node_param_.waypoints_velocity));
  } else {
    switch (search_status) {
      case astar_search::SearchStatus::FAILURE_COLLISION_AT_START:
        RCLCPP_ERROR(
          get_logger(), "Cannot find plan because collision was detected in start position.");
        break;
      case astar_search::SearchStatus::FAILURE_COLLISION_AT_GOAL:
        RCLCPP_ERROR(
          get_logger(), "Cannot find plan because collision was detected in goal position.");
        break;
      case astar_search::SearchStatus::FAILURE_TIMEOUT_EXCEEDED:
        RCLCPP_ERROR(get_logger(), "Cannot find plan because timeout exceeded.");
        break;
      case astar_search::SearchStatus::FAILURE_NO_PATH_FOUND:
        RCLCPP_ERROR(get_logger(), "Cannot find plan.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "SearchStatus not handled.");
        break;
    }
  }

  return astar_search::isSuccess(search_status);
}

bool AstarPlannerNode::isPlanning() const
{
  return state_ == AstarPlannerState::PLANNING;
}
void AstarPlannerNode::startPlanning()
{
  state_ = AstarPlannerState::PLANNING;
}
void AstarPlannerNode::stopPlanning()
{
  state_ = AstarPlannerState::IDLE;
}

void AstarPlannerNode::reset()
{
  trajectory_ = usv_msgs::msg::Trajectory{};
}

geometry_msgs::msg::TransformStamped AstarPlannerNode::getTransform(
  const std::string & target, const std::string & source)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      target, source, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void AstarPlannerNode::visualizeTrajectory()
{
  auto debug_pose_array_trajectory = geometry_msgs::msg::PoseArray();

  debug_pose_array_trajectory.header = trajectory_.header;

  for (const auto & trajectory_pose : trajectory_.points) {
    auto pose = trajectory_pose.pose;
    debug_pose_array_trajectory.poses.push_back(pose);
  }

  // publish visualization friendly pose array
  pose_array_trajectory_debug_pub_->publish(debug_pose_array_trajectory);

  // publish trajectory as-is
  trajectory_debug_pub_->publish(trajectory_);
}

}  // namespace astar_planner
}  // namespace planning
}  // namespace usv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usv::planning::astar_planner::AstarPlannerNode)
