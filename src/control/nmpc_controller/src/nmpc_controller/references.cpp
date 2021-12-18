#include <algorithm>
#include <stdexcept>

#include "nmpc_controller/nmpc_controller.hpp"
#include <motion_common/motion_common.hpp>

namespace motion
{
namespace control
{
namespace nmpc_controller
{
// double
using AcadosReal = real_t;

////////////////////////////////////////////////////////////////////////////////

constexpr auto HORIZON = N;
// State variables
static_assert(NX == 6, "Unexpected num of state variables");
// constexpr auto IDX_X = 0U;
// constexpr auto IDX_Y = 1U;
constexpr auto IDX_HEADING = 2U;
// constexpr auto IDX_VEL_LONG = 3U;
static_assert(NYN == 6, "Unexpected number of terminal reference variables");
constexpr auto IDYN_X = 0U;
constexpr auto IDYN_Y = 1U;
constexpr auto IDYN_HEADING = 2U;
constexpr auto IDYN_VEL_LONG = 3U;
constexpr auto IDYN_VEL_TRAN = 4U;
constexpr auto IDYN_VEL_ANGULAR = 5U;
// Reference variable indices
static_assert(NY == 8, "Unexpected number of reference variables");
constexpr auto IDY_X = 0U;
constexpr auto IDY_Y = 1U;
constexpr auto IDY_HEADING = 2U;
constexpr auto IDY_VEL_LONG = 3U;
constexpr auto IDY_VEL_TRAN = 4U;
constexpr auto IDY_VEL_ANGULAR = 5U;
constexpr auto IDY_LEFT_CMD = 6U;
constexpr auto IDY_RIGHT_CMD = 7U;

////////////////////////////////////////////////////////////////////////////////
void NmpcController::apply_weights(const OptimizationConfig & cfg)
{
  apply_nominal_weights(cfg.nominal(), Index{}, HORIZON);
  set_terminal_weights(cfg.terminal());
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::zero_terminal_weights()
{
  static_assert(NYN == 6, "Unexpected number of terminal reference variables");
  std::array<AcadosReal, NYN * NYN> WN{};
  ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, HORIZON, "W", WN.data());
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::set_terminal_weights(const StateWeight & cfg)
{
  static_assert(NYN == 6, "Unexpected number of terminal reference variables");
  std::array<AcadosReal, NYN * NYN> WN{};
  WN[(IDYN_X * NYN) + IDYN_X] = static_cast<AcadosReal>(cfg.pose_x());
  WN[(IDYN_Y * NYN) + IDYN_Y] = static_cast<AcadosReal>(cfg.pose_y());
  WN[(IDYN_HEADING * NYN) + IDYN_HEADING] = static_cast<AcadosReal>(cfg.heading());
  WN[(IDYN_VEL_LONG * NYN) + IDYN_VEL_LONG] = static_cast<AcadosReal>(cfg.longitudinal_velocity());
  WN[(IDYN_VEL_TRAN * NYN) + IDYN_VEL_TRAN] = static_cast<AcadosReal>(cfg.lateral_velocity());
  WN[(IDYN_VEL_ANGULAR * NYN) + IDYN_VEL_ANGULAR] = static_cast<AcadosReal>(cfg.yaw_rate());
  ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, HORIZON, "W", WN.data());
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::apply_nominal_weights(const StateWeight & cfg, const Index start, Index end)
{
  static_assert(sizeof(std::size_t) >= sizeof(Index), "static cast might truncate");
  end = std::min(static_cast<std::size_t>(end), HORIZON);
  if (start > end) {
    throw std::logic_error{"Inconsistent bounds: apply! There's likely an indexing bug somewhere"};
  }
  static_assert(NY == 8, "Unexpected number of reference variables");
  std::array<AcadosReal, NY * NY> W{};
  W[(IDY_X * NY) + IDY_X] = static_cast<AcadosReal>(cfg.pose_x());
  W[(IDY_Y * NY) + IDY_Y] = static_cast<AcadosReal>(cfg.pose_y());
  W[(IDY_HEADING * NY) + IDY_HEADING] = static_cast<AcadosReal>(cfg.heading());
  W[(IDY_VEL_LONG * NY) + IDY_VEL_LONG] = static_cast<AcadosReal>(cfg.longitudinal_velocity());
  W[(IDY_VEL_TRAN * NY) + IDY_VEL_TRAN] = static_cast<AcadosReal>(cfg.lateral_velocity());
  W[(IDY_VEL_ANGULAR * NY) + IDY_VEL_ANGULAR] = static_cast<AcadosReal>(cfg.yaw_rate());
  W[(IDY_LEFT_CMD * NY) + IDY_LEFT_CMD] = static_cast<AcadosReal>(cfg.left_cmd());
  W[(IDY_RIGHT_CMD * NY) + IDY_RIGHT_CMD] = static_cast<AcadosReal>(cfg.right_cmd());
  for (Index i = start; i < end; ++i) {
    ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, i, "W", W.data());
  }
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::apply_terminal_weights(const Index idx)
{
  if (idx >= HORIZON) {
    throw std::logic_error{"Out of bounds terminal weight index"};
  }
  const auto & param = get_config().optimization_param().terminal();
  apply_nominal_weights(param, idx, idx + 1U);
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::zero_nominal_weights(const Index start, Index end)
{
  static_assert(sizeof(std::size_t) >= sizeof(Index), "static cast might truncate");
  end = std::min(HORIZON, static_cast<std::size_t>(end));
  if (start > end) {
    throw std::logic_error{"Inconsistent bounds: zero! There's likely an indexing bug somewhere"};
  }
  static_assert(NY == 8, "Unexpected number of reference variables");
  std::array<AcadosReal, NY * NY> W{};
  for (Index i = start; i < end; ++i) {
    ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, i, "W", W.data());
  }
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::advance_problem(const Index count)
{
  if (HORIZON <= count) {
    throw std::logic_error{"Advancing count too high! Likely indexing bug somewhere"};
  }
  // some re-used data structures
  std::array<AcadosReal, NX> initial_x{};
  std::array<AcadosReal, NU> initial_u{};
  std::array<AcadosReal, NX> last_x{};
  std::array<AcadosReal, NU> last_u{};
  for (auto i = count; i < HORIZON; ++i) {
    {
      const auto idx = i + decltype(i){1};
      ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, idx, "x", last_x.data());
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, idx - count, "x", last_x.data());
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, idx, "x", initial_x.data());
    }
    {
      const auto idx = i;
      ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, idx, "u", last_u.data());
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, idx - count, "u", last_u.data());
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, idx, "u", initial_u.data());
    }
    ocp_nlp_cost_model_set(
      m_nlp_config, m_nlp_dims, m_nlp_in, i - count, "yref", m_last_reference.data() + (NY * i));
  }
  // modify last reference
  std::copy(
    m_last_reference.begin() + NY * count, m_last_reference.end(), m_last_reference.begin());
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::backfill_reference(const Index count)
{
  static_assert(sizeof(std::size_t) >= sizeof(Index), "static cast might truncate");
  if (HORIZON <= count) {
    throw std::logic_error{"Backfill count too high! Likely indexing bug somewhere"};
  }
  // Start filling from count before the end
  const auto ref_start = HORIZON - count;
  // start pulling from the trajectory N - count from the current point
  const auto max_pts = static_cast<std::size_t>(get_reference_trajectory().points.size());
  const auto curr_idx = get_current_state_temporal_index();
  const auto traj_start = std::min(curr_idx + ref_start, max_pts);
  // Try to pull up to count
  const auto traj_end = std::min(traj_start + count, max_pts);
  const auto safe_count = traj_end - traj_start;
  // Pull references from trajectory
  set_reference(get_reference_trajectory(), ref_start, traj_start, safe_count);
  // Zero out remainder
  if (safe_count < count) {
    const auto remainder = count - safe_count;
    zero_nominal_weights(HORIZON - remainder, HORIZON);
    apply_terminal_weights(HORIZON - (remainder + 1));
  }
  // Set the terminal reference
  if (traj_start + count < max_pts) {
    set_terminal_reference(get_reference_trajectory().points[traj_start + count]);
  } else {
    zero_terminal_weights();
  }
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::set_reference(
  const Trajectory & traj, const Index y_start, const Index traj_start, const Index count)
{
  if ((y_start + count) > HORIZON || (traj_start + count) > traj.points.size()) {
    throw std::logic_error{"set_reference would go out of bounds! Indexing bug likely!"};
  }
  std::array<AcadosReal, NY> y{};
  for (auto i = Index{}; i < count; ++i) {
    const auto & pt = traj.points[traj_start + i];
    const auto ydx = y_start + i;
    y[IDY_X] = static_cast<AcadosReal>(pt.pose.position.x);
    y[IDY_Y] = static_cast<AcadosReal>(pt.pose.position.y);
    y[IDY_VEL_LONG] = static_cast<AcadosReal>(pt.longitudinal_velocity_mps);
    y[IDY_HEADING] = static_cast<AcadosReal>(motion_common::to_angle(pt.pose.orientation));
    y[IDY_VEL_TRAN] = static_cast<AcadosReal>(pt.lateral_velocity_mps);
    y[IDYN_VEL_ANGULAR] = static_cast<AcadosReal>(pt.heading_rate_rps);
    ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, ydx, "yref", y.data());
    // modify last reference
    std::copy(y.cbegin(), y.cend(), m_last_reference.begin() + ydx * NY);
  }
}

////////////////////////////////////////////////////////////////////////////////
bool NmpcController::ensure_reference_consistency(Index horizon)
{
  static_assert(sizeof(std::size_t) >= sizeof(Index), "static cast might truncate");
  horizon = std::min(static_cast<std::size_t>(horizon), HORIZON);
  std::array<AcadosReal, NX> last_state{};
  ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, 0, "x", last_state.data());
  auto last_angle = last_state.at(IDX_HEADING);
  auto err = AcadosReal{};
  const auto fn = [&last_angle, &err](auto & ref) {
    const auto dth = ref - last_angle;
    const auto diff = std::atan2(std::sin(dth), std::cos(dth));
    const auto ref_old = ref;
    ref = last_angle + diff;
    err += std::fabs(ref - ref_old);
    last_angle = ref;
  };
  for (auto i = Index{}; i <= horizon; ++i) {
    const auto idx = NY * i;
    auto heading = m_last_reference.at(idx + IDY_HEADING);
    fn(heading);
  }
  constexpr auto PI = AcadosReal{usv::common::types::PI};
  return err > PI;
}

////////////////////////////////////////////////////////////////////////////////
void NmpcController::set_terminal_reference(const Point & pt)
{
  std::array<AcadosReal, NYN> yN{};
  yN[IDYN_X] = static_cast<AcadosReal>(pt.pose.position.x);
  yN[IDYN_Y] = static_cast<AcadosReal>(pt.pose.position.y);
  yN[IDYN_VEL_LONG] = static_cast<AcadosReal>(pt.longitudinal_velocity_mps);
  yN[IDYN_HEADING] = static_cast<AcadosReal>(motion_common::to_angle(pt.pose.orientation));
  yN[IDYN_VEL_TRAN] = static_cast<AcadosReal>(pt.lateral_velocity_mps);
  yN[IDYN_VEL_ANGULAR] = static_cast<AcadosReal>(pt.heading_rate_rps);
  ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, HORIZON, "yref", yN.data());
  // modify last terminal reference
  std::copy(yN.cbegin(), yN.cend(), m_last_terminal_reference.begin());
}

////////////////////////////////////////////////////////////////////////////////
const Trajectory & NmpcController::handle_new_trajectory(const Trajectory & trajectory)
{
  static_assert(sizeof(std::size_t) >= sizeof(Index), "static cast might truncate");
  if (m_interpolated_trajectory) {
    using motion_common::sample;
    sample(trajectory, *m_interpolated_trajectory, solver_time_step);
  }
  const auto & traj = m_interpolated_trajectory ? *m_interpolated_trajectory : trajectory;
  const auto t_max = std::min(static_cast<std::size_t>(traj.points.size()), HORIZON);
  set_reference(traj, Index{}, Index{}, t_max);
  const auto & weights = get_config().optimization_param();
  apply_nominal_weights(weights.nominal(), Index{}, t_max);

  // Set terminal for infinite horizon control, and unset for finite horizon
  if (t_max < HORIZON) {
    // set remaining unused points from t_max to HORIZON as zero_nominal
    zero_nominal_weights(t_max, HORIZON);
  }
  // Set last reference point (with special weights) to one past whatever
  // the hardcoded optimization horizon is
  if (t_max >= traj.points.size()) {
    zero_terminal_weights();  // no terminal set
    apply_terminal_weights(traj.points.size() - 1U);
  } else {  // traj.points.size() > t_max
    // size is at least Horizon + 1  and tmax==HORIZON, used as reference
    set_terminal_reference(traj.points[t_max]);
  }
  m_last_reference_index = {};

  return traj;
}
}  // namespace nmpc_controller
}  // namespace control
}  // namespace motion
