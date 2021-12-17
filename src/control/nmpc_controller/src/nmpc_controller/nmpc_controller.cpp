#include "nmpc_controller/nmpc_controller.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>


namespace motion
{
namespace control
{
namespace nmpc_controller
{
using AcadosReal = real_t;

////////////////////////////////////////////////////////////////////////////////
constexpr auto HORIZON = static_cast<std::size_t>(N);
// State variable indices
static_assert(NX == 6, "Unexpected num of state variables");
constexpr auto IDX_X = 0U;
constexpr auto IDX_Y = 1U;
constexpr auto IDX_HEADING = 2U;
constexpr auto IDX_VEL_LONG = 3U;
constexpr auto IDX_VEL_TRAN = 4U;
constexpr auto IDX_VEL_ANGULAR = 5U;
// Control variable indices
static_assert(NU == 2, "Unexpected num of control variables");
constexpr auto IDX_LEFT_CMD = 0U;
constexpr auto IDX_RIGHT_CMD = 1U;

constexpr std::chrono::nanoseconds NmpcController::solver_time_step;

////////////////////////////////////////////////////////////////////////////////
NmpcController::NmpcController(const Config & config)
: ControllerBase{config.behavior()},
  m_config{config},
  m_computed_trajectory{rosidl_runtime_cpp::MessageInitialization::ALL},
  m_acados_ocp_capsule{autousv_acados_create_capsule()}
{
  acados_init();
  if (config.do_interpolate()) {
    m_interpolated_trajectory = std::make_unique<Trajectory>();
    m_interpolated_trajectory->points.reserve(Trajectory::CAPACITY);
  }
  m_computed_trajectory.points.reserve(Trajectory::CAPACITY);
  apply_config(m_config);
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::acados_init()
{
  double * new_time_steps{nullptr};
  const auto status =
    autousv_acados_create_with_discretization(m_acados_ocp_capsule, N, new_time_steps);
  if (0 != status) {
    std::string err_str{"autousv_acados_create() error: ", std::string::allocator_type{}};
    err_str += std::to_string(status);
    throw std::runtime_error{err_str};
  }
  {
    m_nlp_config = autousv_acados_get_nlp_config(m_acados_ocp_capsule);
    m_nlp_dims = autousv_acados_get_nlp_dims(m_acados_ocp_capsule);
    m_nlp_in = autousv_acados_get_nlp_in(m_acados_ocp_capsule);
    m_nlp_out = autousv_acados_get_nlp_out(m_acados_ocp_capsule);
    m_nlp_solver = autousv_acados_get_nlp_solver(m_acados_ocp_capsule);
  }
}

////////////////////////////////////////////////////////////////////////////////
const Config & NmpcController::get_config() const
{
  return m_config;
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::set_config(const Config & config)
{
  m_config = config;
  ControllerBase::set_base_config(m_config.behavior());
  apply_config(m_config);
}

////////////////////////////////////////////////////////////////////////////////
Command NmpcController::compute_command_impl(const State & state)
{
  const auto current_idx = get_current_state_temporal_index();

  auto cold_start = update_references(current_idx);
  const auto dt = x0_time_offset(state, current_idx);
  initial_conditions(predict(state.state, dt));
  {
    static_assert(sizeof(std::size_t) >= sizeof(Index), "static cast might truncate");
    // This HAS to happen after initial conditions; relies on x0 being set
    // In addition, this has to run every iteration, since there's no guarantee of
    // smoothness in the received state
    const auto max_pts = get_reference_trajectory().points.size();
    const auto horizon = std::min(static_cast<std::size_t>(max_pts - current_idx), HORIZON);
    cold_start = ensure_reference_consistency(horizon) || cold_start;
    // Consider different ways of updating initial guess for reference update
  }
  if (cold_start) {
    std::array<AcadosReal, NU> initial{};
    for (auto i = Index{}; i <= HORIZON; ++i) {
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, i, "u", initial.data());
    }
  }
  solve();
  // Get result
  return interpolated_command(dt);
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::solve()
{
  const auto solve_ret = autousv_acados_solve(m_acados_ocp_capsule);
  if (0 != solve_ret) {
    std::string err_str{"Solver error: ", std::string::allocator_type{}};
    err_str += std::to_string(solve_ret);
    throw std::runtime_error{err_str};
  }
}

////////////////////////////////////////////////////////////////////////////////
bool NmpcController::update_references(const Index current_idx)
{
  const auto cold_start = Index{} == current_idx;
  // Roll forward previous solutions, references; backfill references or prune weights
  if (!cold_start) {
    const auto advance_idx = current_idx - m_last_reference_index;
    advance_problem(advance_idx);
    const auto max_pts = get_reference_trajectory().points.size();
    if (max_pts - current_idx >= HORIZON) {
      backfill_reference(advance_idx);
    } else {
      const auto receded_horizon = max_pts - current_idx;
      apply_terminal_weights(receded_horizon - 1);
      zero_nominal_weights(receded_horizon, receded_horizon + advance_idx);
      zero_terminal_weights();
    }

    // Set index *before* potential throw statements
    m_last_reference_index = current_idx;
  }
  return cold_start;
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::initial_conditions(const Point & state)
{
  std::array<AcadosReal, NX> idx_bx0{
    IDX_X, IDX_Y, IDX_HEADING, IDX_VEL_LONG, IDX_VEL_TRAN, IDX_VEL_ANGULAR};
  ocp_nlp_constraints_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, 0, "idxbx", idx_bx0.data());
  // Set x0
  std::array<AcadosReal, NX> x0{};
  x0[IDX_X] = static_cast<AcadosReal>(state.x);
  x0[IDX_Y] = static_cast<AcadosReal>(state.y);
  x0[IDX_HEADING] = static_cast<AcadosReal>(motion_common::to_angle(state.heading));
  x0[IDX_VEL_LONG] = static_cast<AcadosReal>(state.longitudinal_velocity_mps);
  x0[IDX_VEL_TRAN] = static_cast<AcadosReal>(state.lateral_velocity_mps);
  x0[IDX_VEL_ANGULAR] = static_cast<AcadosReal>(state.heading_rate_rps);
  ocp_nlp_constraints_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, 0, "lbx", x0.data());
  ocp_nlp_constraints_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, 0, "ubx", x0.data());
}

////////////////////////////////////////////////////////////////////////////////
std::chrono::nanoseconds NmpcController::x0_time_offset(const State & state, Index idx)
{
  using time_utils::from_message;
  const auto & traj = get_reference_trajectory();
  // What time stamp of x0 should be
  const auto t0 =
    (from_message(traj.header.stamp) + from_message(traj.points[idx].time_from_start));
  const auto dt = t0 - from_message(state.header.stamp);
  return dt;
}

////////////////////////////////////////////////////////////////////////////////
Command NmpcController::interpolated_command(const std::chrono::nanoseconds x0_time_offset)
{
  // If I roll backwards, then the actual now() time is forward
  const auto step = solver_time_step;
  auto dt_ = get_config().control_lookahead_duration() - x0_time_offset;
  const auto max_dt = step * (HORIZON - 1U);
  if (dt_ > max_dt) {
    dt_ = max_dt;
  }
  // Compute count
  // At most second from last for interpolation
  const auto count = std::min(dt_ / step, static_cast<decltype(dt_)::rep>(HORIZON - 2U));
  const auto dt = dt_ - (count * step);
  using std::chrono::duration;
  using std::chrono::duration_cast;
  const auto t = duration_cast<duration<float>>(dt) / duration_cast<duration<float>>(step);

  std::array<AcadosReal, NU> u{};
  Command ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  {
    // interpolation
    const auto idx = count;
    ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, idx, "u", u.data());
    const auto left_cmd0 = static_cast<Real>(u[IDX_LEFT_CMD]);
    const auto right_cmd0 = static_cast<Real>(u[IDX_RIGHT_CMD]);
    const auto jdx = count + 1U;
    ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, jdx, "u", u.data());
    const auto left_cmd1 = static_cast<Real>(u[IDX_LEFT_CMD]);
    const auto right_cmd1 = static_cast<Real>(u[IDX_RIGHT_CMD]);
    ret.left_cmd = motion_common::interpolate(left_cmd0, left_cmd1, t);
    ret.right_cmd = motion_common::interpolate(right_cmd0, right_cmd1, t);
  }

  if (!std::isfinite(ret.left_cmd) || !std::isfinite(ret.right_cmd)) {
    throw std::runtime_error{"interpolation failed, result is not finite (NaN/Inf)"};
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
const NmpcController::Trajectory & NmpcController::get_computed_trajectory() const noexcept
{
  auto & traj = m_computed_trajectory;
  traj.header = get_reference_trajectory().header;
  traj.points.resize(HORIZON);
  std::array<AcadosReal, NX> x{};
  for (std::size_t i = {}; i < HORIZON; ++i) {
    auto & pt = traj.points[i];
    ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, i, "x", x.data());
    pt.x = static_cast<Real>(x[IDX_X]);
    pt.y = static_cast<Real>(x[IDX_Y]);
    pt.longitudinal_velocity_mps = static_cast<Real>(x[IDX_VEL_LONG]);
    pt.lateral_velocity_mps = static_cast<Real>(x[IDX_VEL_TRAN]);
    const auto heading = static_cast<Real>(x[IDX_HEADING]);
    pt.heading = motion_common::from_angle(heading);
    pt.heading_rate_rps = static_cast<Real>(x[IDX_VEL_ANGULAR]);
  }
  return m_computed_trajectory;
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::apply_config(const Config & cfg)
{
  apply_parameters(cfg.vehicle_param(), cfg.env_forces());
  apply_bounds(cfg.limits());
  apply_weights(cfg.optimization_param());
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::apply_parameters(
  const VehicleConfig & cfg, const EnvironmentForces & ef) noexcept
{
  static_assert(NP == 10U, "Num online parameters is not expected value!");
  std::array<AcadosReal, NP> p{
    cfg.mass11(),
    cfg.mass22(),
    cfg.mass33(),
    cfg.damping_11(),
    cfg.damping_22(),
    cfg.damping_33(),
    cfg.length_cg_thrusters_lateral_m(),
    ef.tau_env_u,
    ef.tau_env_v,
    ef.tau_env_r};
  for (std::size_t i = {}; i < HORIZON; ++i) {
    autousv_acados_update_params(m_acados_ocp_capsule, i, p.data(), NP);
  }
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::apply_bounds(const LimitsConfig & cfg) noexcept
{
  constexpr auto NUM_CTRL_CONSTRAINTS = 2U;
  constexpr auto NUM_STATE_CONSTRAINTS = 3U;
  constexpr auto idx_left = 0U;
  constexpr auto idx_right = 1U;
  std::array<AcadosReal, NUM_CTRL_CONSTRAINTS> idx_bu{idx_left, idx_right};
  std::array<AcadosReal, NUM_CTRL_CONSTRAINTS> ctrl_lb{
    cfg.thruster_force().min(), cfg.thruster_force().min()};
  std::array<AcadosReal, NUM_CTRL_CONSTRAINTS> ctrl_ub{
    cfg.thruster_force().max(), cfg.thruster_force().max()};
  constexpr auto idx_u = 3U;
  constexpr auto idx_v = 4U;
  constexpr auto idx_r = 5U;
  std::array<AcadosReal, NUM_STATE_CONSTRAINTS> idx_bx{idx_u, idx_v, idx_r};
  std::array<AcadosReal, NUM_STATE_CONSTRAINTS> states_lb{
    cfg.longitudinal_velocity().min(), cfg.lateral_velocity().min(), cfg.yaw_rate().min()};
  std::array<AcadosReal, NUM_STATE_CONSTRAINTS> states_ub{
    cfg.longitudinal_velocity().max(), cfg.lateral_velocity().max(), cfg.yaw_rate().max()};
  for (std::size_t i = {}; i < HORIZON; ++i) {
    {
      // range: 0..HORIZON
      ocp_nlp_constraints_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, i, "idxbu", idx_bu.data());
      ocp_nlp_constraints_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, i, "lbu", ctrl_lb.data());
      ocp_nlp_constraints_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, i, "ubu", ctrl_ub.data());
    }
    {
      // These are different from the general state constraints because not all states
      // have constraints
      // range: 1..=HORIZON
      const auto idx = i + decltype(i){1};
      ocp_nlp_constraints_model_set(
        m_nlp_config, m_nlp_dims, m_nlp_in, idx, "idxbx", idx_bx.data());
      ocp_nlp_constraints_model_set(
        m_nlp_config, m_nlp_dims, m_nlp_in, idx, "lbx", states_lb.data());
      ocp_nlp_constraints_model_set(
        m_nlp_config, m_nlp_dims, m_nlp_in, idx, "ubx", states_ub.data());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
bool NmpcController::check_new_trajectory(const Trajectory & trajectory) const
{
  // Check that all heading values are valid (i.e. are normalized 2D quaternions)
  if (!motion_common::heading_ok(trajectory)) {
    return false;
  }
  // If interpolating, no checks yet
  if (get_config().do_interpolate()) {
    return true;
  }
  for (Index idx = {}; idx < trajectory.points.size(); ++idx) {
    const auto & pt = trajectory.points[idx];
    // std::chrono::duration::abs is C++17
    const auto t0 = time_utils::from_message(trajectory.header.stamp);
    const auto ref = (idx * solver_time_step) + t0;
    const auto dt = get_config().sample_period_tolerance();
    const auto ub = ref + dt;
    const auto lb = ref - dt;
    const auto t = time_utils::from_message(pt.time_from_start) + t0;
    if ((t < lb) || (t > ub)) {
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
std::string NmpcController::name() const
{
  return std::string{"HPIPM solver", std::allocator<char>{}};
}

////////////////////////////////////////////////////////////////////////////////
Index NmpcController::get_compute_iterations() const
{
  Index sqp_iter{};
  ocp_nlp_get(m_nlp_config, m_nlp_solver, "sqp_iter", &sqp_iter);
  return sqp_iter;
}

////////////////////////////////////////////////////////////////////////////////
NmpcController::~NmpcController()
{
  // free solver
  int status = autousv_acados_free(m_acados_ocp_capsule);
  if (0 != status) {
    std::cerr << "autousv_acados_free() returned status " << status << ".\n";
  }
  // free solver capsule
  status = autousv_acados_free_capsule(m_acados_ocp_capsule);
  if (0 != status) {
    std::cerr << "autousv_acados_free_capsule() returned status " << status << ".\n";
  }
}

}  // namespace nmpc_controller
}  // namespace control
}  // namespace motion
