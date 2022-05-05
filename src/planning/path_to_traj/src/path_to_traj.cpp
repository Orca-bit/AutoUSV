//
// Created by liuhao on 22-5-5.
//

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <time_utils/time_utils.hpp>

#include <nav_msgs/msg/path.hpp>
#include <usv_msgs/msg/trajectory.hpp>
#include <usv_msgs/msg/trajectory_point.hpp>


class PathToTrajNode : public rclcpp::Node
{
private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr& path)
  {
    auto traj = usv_msgs::msg::Trajectory{};
    traj.header = path->header;
    {
      using time_utils::to_message;
      traj.header.stamp = to_message(std::chrono::system_clock::now());
    }
    const auto size =
      std::min(static_cast<size_t>(usv_msgs::msg::Trajectory::CAPACITY), path->poses.size());
    using namespace std::chrono_literals;
    auto time = 0ns;
    for (size_t i = 0; i < size; ++i) {
      auto traj_point = usv_msgs::msg::TrajectoryPoint{};
      if (i != 0) {
        const auto dt =
          sqrt(
            pow(path->poses[i].pose.position.x - path->poses[i - 1].pose.position.x, 2) +
            pow(path->poses[i].pose.position.y - path->poses[i - 1].pose.position.y, 2)) /
          2.0;
        time += std::chrono::duration<int64_t, std::nano>(static_cast<int64_t>(dt * 1e9));
      }
      {
        using time_utils::to_message;
        traj_point.time_from_start = to_message(time);
      }
      traj_point.pose = path->poses[i].pose;
      traj_point.longitudinal_velocity_mps = 2.0;
      traj_point.lateral_velocity_mps = 0.0;
      traj_point.heading_rate_rps = 0.0;
      traj_point.acceleration_mps2 = 0.0;
      traj.points.emplace_back(traj_point);
    }

    traj_publisher_->publish(traj);
  }


public:
  PathToTrajNode() : Node("path_to_traj_node")
  {
    traj_publisher_ = this->create_publisher<usv_msgs::msg::Trajectory>("usv/traj", 10);
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
      "path", rclcpp::QoS{10U}, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_callback(msg);
      });
  }

private:
  rclcpp::Publisher<usv_msgs::msg::Trajectory>::SharedPtr traj_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajNode>());
  rclcpp::shutdown();
  return 0;
}
