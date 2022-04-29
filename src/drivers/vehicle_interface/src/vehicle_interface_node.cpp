#include "vehicle_interface/vehicle_interface_node.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <common/types.hpp>
#include <time_utils/time_utils.hpp>

using usv::common::types::bool8_t;
using usv::common::types::float32_t;
using usv::common::types::float64_t;


namespace usv
{
namespace drivers
{
namespace vehicle_interface
{

VehicleControlCommand::SharedPtr m_msg;
std::shared_ptr<PlatformInterface> m_interface;
std::mutex msg_mut, interface_mut;

////////////////////////////////////////////////////////////////////////////////
// void VehicleInterfaceNode::send_cmd_loop(std::shared_ptr<PlatformInterface> interface)
void send_cmd_loop()
{
  auto interface = m_interface;
  while (true) {
    if (interface == nullptr || m_msg == nullptr || !msg_mut.try_lock()) {
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
      continue;
    }
    auto msg = *m_msg;
    msg_mut.unlock();
    std::lock_guard<std::mutex> lk{interface_mut};
    interface->send_control_command(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
VehicleInterfaceNode::VehicleInterfaceNode(std::string && node_name) : Node{node_name}
{
  // Helper functions
  const auto topic_num_matches_from_param = [this](const auto param) {
    const auto name_param = declare_parameter(param);
    return TopicNumMatches{name_param.template get<std::string>()};
  };
  const auto time = [this](const auto param_name) -> std::chrono::milliseconds {
    const auto count_ms = declare_parameter(param_name).template get<int64_t>();
    return std::chrono::milliseconds{count_ms};
  };

  set_interface(std::make_shared<PlatformInterface>(
    topic_num_matches_from_param("left_thruster_usb_name").topic,
    topic_num_matches_from_param("right_thruster_usb_name").topic));
  // "/dev/ttyUSB0",
  // "/dev/ttyUSB1"));

  // Actually init
  init(
    topic_num_matches_from_param("control_command"),
    // TopicNumMatches{"basic"},
    TopicNumMatches{"left_motor_report1"},
    TopicNumMatches{"right_motor_report1"},
    TopicNumMatches{"left_motor_report2"},
    TopicNumMatches{"right_motor_report2"},
    time("cycle_time_ms"));
  // std::chrono::milliseconds{10});
  m_th = std::thread(send_cmd_loop);
}

void VehicleInterfaceNode::set_interface(std::shared_ptr<PlatformInterface> && interface) noexcept
{
  m_interface = std::forward<std::shared_ptr<PlatformInterface> &&>(interface);
  RCLCPP_INFO_STREAM(
    logger(),
    "left_thruster_usb_name: " << m_interface->get_left_usb_port_name()
                               << ", right_thruster_usb_name: "
                               << m_interface->get_right_usb_port_name());
}

rclcpp::Logger VehicleInterfaceNode::logger() const noexcept
{
  return get_logger();
}

template <typename T> void VehicleInterfaceNode::on_command_message(const T &)
{
  RCLCPP_WARN_STREAM(logger(), "on_command_message: check the cmd msg type: basic or high_level");
}

template <>
void VehicleInterfaceNode::on_command_message(const usv_msgs::msg::HighLevelControlCommand & msg)
{
  /// not implemented
  if (!m_interface->send_control_command(msg)) {
    on_control_send_failure();
  }
}

////////////////////////////////////////////////////////////////////////////////
template <>
void VehicleInterfaceNode::on_command_message(const usv_msgs::msg::VehicleControlCommand & msg)
{
  const auto stamp = time_utils::from_message(msg.stamp);
  const auto dt = stamp - m_last_command_stamp;

  // Time should not go backwards
  if (dt < std::chrono::nanoseconds::zero()) {
    throw std::domain_error{"Vehicle interface command went backwards in time!"};
  }

  if (dt > std::chrono::nanoseconds::zero()) {
    m_last_command_stamp = stamp;
    // Send
    // if (!m_interface->send_control_command(msg)) {
    //   on_control_send_failure();
    // }
    std::lock_guard<std::mutex> lock{msg_mut};
    // *m_msg = msg;
    m_msg = std::make_shared<VehicleControlCommand>(msg);
  } else {
    RCLCPP_WARN(logger(), "Vehicle interface time did not increase, skipping");
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::init(
  const TopicNumMatches & control_command,
  const TopicNumMatches & left_motor_report1,
  const TopicNumMatches & right_motor_report1,
  const TopicNumMatches & left_motor_report2,
  const TopicNumMatches & right_motor_report2,
  const std::chrono::nanoseconds & cycle_time)
{
  m_cycle_time = cycle_time;
  // Timer
  m_read_timer = create_wall_timer(m_cycle_time, [this]() {
    try {
      read_and_publish();
    } catch (...) {
      on_error(std::current_exception());
    }
  });
  // Make publishers
  m_left_motor_report1 = create_publisher<MotorReport1>(left_motor_report1.topic, rclcpp::QoS{10U});
  m_right_motor_report1 =
    create_publisher<MotorReport1>(right_motor_report1.topic, rclcpp::QoS{10U});
  m_left_motor_report2 = create_publisher<MotorReport2>(left_motor_report2.topic, rclcpp::QoS{10U});
  m_right_motor_report2 =
    create_publisher<MotorReport2>(right_motor_report2.topic, rclcpp::QoS{10U});

  const auto cmd_callback = [this](auto t) -> auto
  {
    using Ptr = typename decltype(t)::SharedPtr;
    return [this](Ptr msg) -> void {
      try {
        on_command_message(*msg);
      } catch (...) {
        on_error(std::current_exception());
      }
    };
  };
  if (control_command.topic == "basic") {
    using BasicControlCommand = usv_msgs::msg::VehicleControlCommand;
    m_command_sub = create_subscription<BasicControlCommand>(
      "basic_cmd", rclcpp::QoS{10U}, cmd_callback(BasicControlCommand{}));
  } else if (control_command.topic == "high_level") {
    using HCC = usv_msgs::msg::HighLevelControlCommand;
    m_command_sub =
      create_subscription<HCC>("high_level_cmd", rclcpp::QoS{10U}, cmd_callback(HCC{}));
  } else {
    throw std::domain_error{"Vehicle interface must have exactly one command subscription"};
  }
  check_invariants();
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::check_invariants()
{
  if (decltype(m_cycle_time)::zero() >= m_cycle_time) {
    throw std::domain_error{"Cycle time must be positive"};
  }
  // Check command sub
  const auto ctrl_not_null =
    mpark::visit([](auto && sub) -> bool8_t { return static_cast<bool8_t>(sub); }, m_command_sub);
  if (!ctrl_not_null) {
    throw std::domain_error{"Vehicle interface must have exactly one command subscription"};
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_control_send_failure()
{
  throw std::runtime_error{"Sending control command failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_read_timeout()
{
  throw std::runtime_error{"Receiving data failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_error(std::exception_ptr eptr) const
{
  try {
    std::rethrow_exception(eptr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), e.what());
  } catch (...) {
    RCLCPP_ERROR(logger(), "VehicleInterface: Unknown error!");
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::read_and_publish()
{
  std::lock_guard<std::mutex> lk{interface_mut};
  {
    auto left_motor_report1 = m_interface->get_left_motor_report_1();
    if (left_motor_report1) {
      m_left_motor_report1->publish(left_motor_report1.value());
      m_interface->reset_left_motor_report1();
    }
  }
  {
    auto right_motor_report1 = m_interface->get_right_motor_report_1();
    if (right_motor_report1) {
      m_right_motor_report1->publish(right_motor_report1.value());
      m_interface->reset_right_motor_report1();
    }
  }
  {
    auto left_motor_report2 = m_interface->get_left_motor_report_2();
    if (left_motor_report2) {
      m_left_motor_report2->publish(left_motor_report2.value());
      m_interface->reset_left_motor_report2();
    }
  }
  {
    auto right_motor_report2 = m_interface->get_right_motor_report_2();
    if (right_motor_report2) {
      m_right_motor_report2->publish(right_motor_report2.value());
      m_interface->reset_right_motor_report2();
    }
  }
}


VehicleInterfaceNode::~VehicleInterfaceNode()
{
  m_th.join();
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv
