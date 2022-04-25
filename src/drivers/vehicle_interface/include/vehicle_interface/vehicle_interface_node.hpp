#ifndef VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
#define VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_

#include <chrono>
#include <exception>
#include <experimental/optional>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>

#include <mpark_variant_vendor/variant.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/platform_interface.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <usv_msgs/msg/high_level_control_command.hpp>
#include <usv_msgs/msg/vehicle_control_command.hpp>

namespace usv
{
namespace drivers
{
namespace vehicle_interface
{
/// Convenience struct for construction
struct TopicNumMatches
{
  std::string topic;
};  // struct TopicNumMatches

/// A node which receives commands and sends them to the vehicle platform, and publishes
/// reports from the vehicle platform
class VEHICLE_INTERFACE_PUBLIC VehicleInterfaceNode : public ::rclcpp::Node
{
public:
  /// ROS 2 parameter constructor
  /// \param[in] node_name The name for the node
  /// \param[in] features Vector of features supported by this vehicle interface
  /// \param[in] options An rclcpp::NodeOptions object
  VehicleInterfaceNode(const std::string & node_name, const rclcpp::NodeOptions & options);

  ~VehicleInterfaceNode() override;


  /// Set the vehicle-specific PlatformInterface
  void set_interface(std::shared_ptr<PlatformInterface> && interface) noexcept;
  /// Get access to logger
  rclcpp::Logger logger() const noexcept;

  /// Error handling behavior for when sending a control command has failed, default is throwing an
  /// exception, which is caught and turned into a change in the NodeState to ERROR
  static void on_control_send_failure();
  /// Error handling behavior for when receiving data from the vehicle platform has timed out,
  /// default is throwing an exception, which is caught and turned into a change in the NodeState to
  /// ERROR
  static void on_read_timeout();
  /// Handle exception thrown in main loop. Default behavior is to set NodeState to ERROR
  void on_error(std::exception_ptr eptr) const;

private:
  // Helper function called in constructors
  VEHICLE_INTERFACE_LOCAL void init(
    const TopicNumMatches & control_command,
    const TopicNumMatches & left_motor_report1,
    const TopicNumMatches & right_motor_report1,
    const TopicNumMatches & left_motor_report2,
    const TopicNumMatches & right_motor_report2,
    const std::chrono::nanoseconds & cycle_time);

  // Run just before main loop, ensure that all invariants (possibly from child class) are enforced
  VEHICLE_INTERFACE_LOCAL void check_invariants();
  // Read data from vehicle platform for time budget, publish data
  VEHICLE_INTERFACE_LOCAL void read_and_publish();
  // Core loop for different input commands. Specialized differently for each topic type
  template <typename T> VEHICLE_INTERFACE_LOCAL void on_command_message(const T & msg);
  static void send_cmd_loop();

  rclcpp::TimerBase::SharedPtr m_read_timer{nullptr};
  rclcpp::Publisher<MotorReport1>::SharedPtr m_left_motor_report1{nullptr};
  rclcpp::Publisher<MotorReport1>::SharedPtr m_right_motor_report1{nullptr};
  rclcpp::Publisher<MotorReport2>::SharedPtr m_left_motor_report2{nullptr};
  rclcpp::Publisher<MotorReport2>::SharedPtr m_right_motor_report2{nullptr};

  using BasicSub = rclcpp::Subscription<VehicleControlCommand>::SharedPtr;
  using HighLevelSub = rclcpp::Subscription<HighLevelControlCommand>::SharedPtr;

  mpark::variant<HighLevelSub, BasicSub> m_command_sub{};

  // std::unique_ptr<PlatformInterface> m_interface;
  std::chrono::system_clock::time_point m_last_command_stamp{};
  std::chrono::nanoseconds m_cycle_time{};

  std::thread m_th;

  static VehicleControlCommand::SharedPtr m_msg;
  static std::shared_ptr<PlatformInterface> interface_;
  static std::mutex mut_;
};  // class VehicleInterfaceNode

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace usv

#endif  // VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
