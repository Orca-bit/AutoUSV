#ifndef NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_
#define NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_

// acados
#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
// AutoUSV specific
#include <acados_solver_autousv.h>
// standard
#include <ostream>
#include <string>
// AutoUSV
#include <controller_common/controller_base.hpp>
#include <nmpc_controller/config.hpp>
#include <nmpc_controller/visibility_control.hpp>
// AutoUSV msgs
#include <usv_msgs/msg/trajectory.hpp>
#include <usv_msgs/msg/vehicle_control_command.hpp>
#include <usv_msgs/msg/vehicle_kinematic_state.hpp>

namespace motion
{
namespace control
{
namespace nmpc_controller
{
// Number of intervals in predictive horizon
constexpr size_t N = AUTOUSV_N;
// Number of states
constexpr size_t NX = AUTOUSV_NX;
// Number of control inputs
constexpr size_t NU = AUTOUSV_NU;
// Number of references
constexpr size_t NY = AUTOUSV_NY;
// Number of terminal references
constexpr size_t NYN = AUTOUSV_NYN;
// Number of parameters
constexpr size_t NP = AUTOUSV_NP;

/// \brief an nmpc for usv trajectory following control
class NMPC_CONTROLLER_PUBLIC NmpcController : public controller_common::ControllerBase
{
public:
  constexpr static std::chrono::nanoseconds solver_time_step{std::chrono::milliseconds{100LL}};
  // Disable other constructors, assignment operators
  NmpcController(const NmpcController &) = delete;
  NmpcController(NmpcController &&) noexcept = delete;
  NmpcController & operator=(const NmpcController &) = delete;
  NmpcController & operator=(NmpcController &&) noexcept = delete;

  using Command = usv_msgs::msg::VehicleControlCommand;
  using State = usv_msgs::msg::VehicleKinematicState;
  using Trajectory = usv_msgs::msg::Trajectory;

  /// \brief Constructor
  explicit NmpcController(const Config & config);
  ~NmpcController() override;

  /// Getter for config class
  const Config & get_config() const;
  /// Setter for config class
  void set_config(const Config & config);

  /// Get trajectory plan computed as a result of solving the optimization problem.
  /// This method triggers the rather expensive copy from the solved vector to the trajectory
  /// representation
  const Trajectory & get_computed_trajectory() const noexcept;

  /// Debug printing
  void debug_print(std::ostream & out) const;

  /// Get name of algorithm, for debugging or diagnostic purposes
  std::string name() const override;

  Index get_compute_iterations() const override;

  /// set environment forces
  void set_env_forces(const EnvironmentForces& env_forces) override;

protected:
  /// Checks trajectory
  bool check_new_trajectory(const Trajectory & trajectory) const override;
  /// Sets reference trajectory
  const Trajectory & handle_new_trajectory(const Trajectory & trajectory) override;
  /// Actually compute the control command
  Command compute_command_impl(const State & state) override;

private:
  // init acados
  NMPC_CONTROLLER_LOCAL void acados_init();
  /// Run the solver subroutine
  NMPC_CONTROLLER_LOCAL void solve();
  /// Roll references forward and update weights appropriately, return true if cold start
  NMPC_CONTROLLER_LOCAL bool update_references(Index current_idx);
  /// Set initial conditions for problem
  NMPC_CONTROLLER_LOCAL void initial_conditions(const Point & state);
  /// Compute delta to roll state forward or back to match first reference
  NMPC_CONTROLLER_LOCAL std::chrono::nanoseconds x0_time_offset(const State & state, Index idx);
  /// Compute interpolated command
  NMPC_CONTROLLER_LOCAL Command interpolated_command(std::chrono::nanoseconds x0_time_offset);
  /// Ensure that from x0, reference points evolve smoothly, x0 is assumed to be set
  /// if there's a big change to maintain consistency, return true (force cold start)
  NMPC_CONTROLLER_LOCAL bool ensure_reference_consistency(Index horizon);
  /// Apply config parameters to the optimization solver, does NOT set m_config
  NMPC_CONTROLLER_LOCAL void apply_config(const Config & cfg);
  /// Apply vehicle parameters
  NMPC_CONTROLLER_LOCAL void apply_parameters(
    const VehicleConfig & cfg, const EnvironmentForces & ef) noexcept;
  /// Apply upper/lower bounds to state and control variables
  NMPC_CONTROLLER_LOCAL void apply_bounds(const LimitsConfig & cfg) noexcept;
  /// Apply weights to optimization objective
  NMPC_CONTROLLER_LOCAL void apply_weights(const OptimizationConfig & cfg);
  /// Apply nominal weights for a specified index range
  NMPC_CONTROLLER_LOCAL void apply_nominal_weights(const StateWeight & cfg, Index start, Index end);
  /// Apply terminal weight somewhere in the middle of y, for receding horizon case
  NMPC_CONTROLLER_LOCAL void apply_terminal_weights(Index idx);
  /// Takes points from reference trajectory and applies to reference
  NMPC_CONTROLLER_LOCAL
  void set_reference(const Trajectory & traj, Index y_start, Index traj_start, Index count);
  /// Zero out fields of the nominal weights
  NMPC_CONTROLLER_LOCAL void zero_nominal_weights(Index start, Index end);
  /// Zero out terminal weights
  NMPC_CONTROLLER_LOCAL void zero_terminal_weights();
  /// Set terminal weights
  NMPC_CONTROLLER_LOCAL void set_terminal_weights(const StateWeight & cfg);
  /// Set terminal reference
  NMPC_CONTROLLER_LOCAL void set_terminal_reference(const Point & pt);
  /// Roll solution and reference forward
  NMPC_CONTROLLER_LOCAL void advance_problem(Index count);
  /// Fills the last N trajectory reference points with points from the reference trajectory
  NMPC_CONTROLLER_LOCAL void backfill_reference(Index count);

  Config m_config;
  Trajectory::UniquePtr m_interpolated_trajectory{nullptr};
  mutable Trajectory m_computed_trajectory;
  Index m_last_reference_index{};
  std::array<real_t, NY * N> m_last_reference{};
  std::array<Real, NYN> m_last_terminal_reference{};
  // acados
  autousv_solver_capsule * m_acados_ocp_capsule;
  ocp_nlp_config * m_nlp_config{nullptr};
  ocp_nlp_dims * m_nlp_dims{nullptr};
  ocp_nlp_in * m_nlp_in{nullptr};
  ocp_nlp_out * m_nlp_out{nullptr};
  ocp_nlp_solver * m_nlp_solver{nullptr};
};  // class NmpcController
}  // namespace nmpc_controller
}  // namespace control
}  // namespace motion
#endif  // NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_
