//
// Created by liuhao on 22-4-29.
//
#include <ctime>
#include <experimental/optional>
#include <fstream>
#include <iostream>
#include <string>

#include <motion_common/motion_common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <time_utils/time_utils.hpp>

#include <usv_msgs/msg/motor_report1.hpp>
#include <usv_msgs/msg/vehicle_kinematic_state.hpp>

using State = usv_msgs::msg::VehicleKinematicState;
using MotorReport = usv_msgs::msg::MotorReport1;

class RecordNode : public ::rclcpp::Node
{
public:
  RecordNode();
  ~RecordNode() override;

  void init();

private:
  rclcpp::Subscription<State>::SharedPtr m_sub_gnss{nullptr};
  rclcpp::Subscription<MotorReport>::SharedPtr m_sub_left_motor{nullptr};
  rclcpp::Subscription<MotorReport>::SharedPtr m_sub_right_motor{nullptr};
  std::ofstream gnss_recorder;
  std::ofstream left_motor_recorder;
  std::ofstream right_motor_recorder;

  std::experimental::optional<std::chrono::time_point<std::chrono::system_clock>> m_start_gnss;
  std::experimental::optional<std::chrono::time_point<std::chrono::system_clock>>
    m_start_left_motor;
  std::experimental::optional<std::chrono::time_point<std::chrono::system_clock>>
    m_start_right_motor;
};

RecordNode::RecordNode() : Node("gnss_test")
{
  init();
}

void RecordNode::init()
{
  const auto now = time(0);
  std::string time = ctime(&now);

  gnss_recorder.open("gnss_" + time + ".csv", std::ios::out | std::ios::trunc);
  left_motor_recorder.open("left_motor_" + time + ".csv", std::ios::out | std::ios::trunc);
  right_motor_recorder.open("right_motor_" + time + ".csv", std::ios::out | std::ios::trunc);

  gnss_recorder << "time"
                << "x"
                << "y"
                << "heading"
                << "u"
                << "v"
                << "r" << '\n';
  left_motor_recorder << "time"
                      << "direction"
                      << "power" << '\n';
  right_motor_recorder << "time"
                       << "direction"
                       << "power" << '\n';

  using motion::motion_common::to_angle;
  using time_utils::from_message;

  auto cb1 = [this](State::SharedPtr msg) {
    const auto time_stamp = from_message(msg->header.stamp);
    if (!m_start_gnss) {
      m_start_gnss = time_stamp;
    }
    const auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(time_stamp - *m_start_gnss);
    gnss_recorder << ms.count() << msg->state.pose.position.x << msg->state.pose.position.y
                  << to_angle(msg->state.pose.orientation) << msg->state.longitudinal_velocity_mps
                  << msg->state.lateral_velocity_mps << msg->state.heading_rate_rps << '\n';
  };

  auto cb2 = [this](MotorReport::SharedPtr msg) {
    const auto time_stamp = from_message(msg->stamp);
    if (!m_start_left_motor) {
      m_start_left_motor = time_stamp;
    }
    const auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(time_stamp - *m_start_left_motor);
    left_motor_recorder << ms.count() << static_cast<int>(msg->rotate_direction) << msg->motor_power
                        << '\n';
  };

  auto cb3 = [this](MotorReport::SharedPtr msg) {
    const auto time_stamp = from_message(msg->stamp);
    if (!m_start_right_motor) {
      m_start_right_motor = time_stamp;
    }
    const auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(time_stamp - *m_start_right_motor);
    right_motor_recorder << ms.count() << static_cast<int>(msg->rotate_direction)
                         << msg->motor_power << '\n';
  };

  m_sub_gnss = create_subscription<State>("gnss_odom", rclcpp::QoS{10U}, cb1);
  m_sub_left_motor = create_subscription<MotorReport>("left_motor_report1", rclcpp::QoS{10U}, cb2);
  m_sub_right_motor =
    create_subscription<MotorReport>("right_motor_report1", rclcpp::QoS{10U}, cb3);
}

RecordNode::~RecordNode()
{
  gnss_recorder.close();
  left_motor_recorder.close();
  right_motor_recorder.close();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RecordNode>());
  rclcpp::shutdown();
  return 0;
}