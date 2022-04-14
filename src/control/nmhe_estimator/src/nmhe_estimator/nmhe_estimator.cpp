//
// Created by liuhao on 2022/1/11.
//

#include <nmhe_estimator/nmhe_estimator.hpp>
#include <time_utils/time_utils.hpp>

namespace motion
{
namespace control
{
namespace nmhe_estimator
{
using AcadosReal = real_t;

constexpr std::chrono::nanoseconds NmheEstimator::solver_time_step;

NmheEstimator::NmheEstimator(Config config)
: m_config(config), m_acados_ocp_capsule{mhe_autousv_ode_with_noisy_param_acados_create_capsule()}
{
  acados_init();
}

NmheEstimator::~NmheEstimator()
{
  // free solver
  int status = mhe_autousv_ode_with_noisy_param_acados_free(m_acados_ocp_capsule);
  if (0 != status) {
    std::cerr << "mhe_autousv_ode_with_noisy_param_acados_free() returned status " << status
              << ".\n";
  }
  // free solver capsule
  status = mhe_autousv_ode_with_noisy_param_acados_free_capsule(m_acados_ocp_capsule);
  if (0 != status) {
    std::cerr << "mhe_autousv_ode_with_noisy_param_acados_free_capsule() returned status " << status
              << ".\n";
  }
}

void NmheEstimator::acados_init()
{
  double * new_time_steps{nullptr};
  const auto status = mhe_autousv_ode_with_noisy_param_acados_create_with_discretization(
    m_acados_ocp_capsule, N, new_time_steps);
  if (0 != status) {
    std::string err_str{
      "mhe_autousv_ode_with_noisy_param_acados_create() error: ", std::string::allocator_type{}};
    err_str += std::to_string(status);
    throw std::runtime_error{err_str};
  }
  {
    m_nlp_config = mhe_autousv_ode_with_noisy_param_acados_get_nlp_config(m_acados_ocp_capsule);
    m_nlp_dims = mhe_autousv_ode_with_noisy_param_acados_get_nlp_dims(m_acados_ocp_capsule);
    m_nlp_in = mhe_autousv_ode_with_noisy_param_acados_get_nlp_in(m_acados_ocp_capsule);
    m_nlp_out = mhe_autousv_ode_with_noisy_param_acados_get_nlp_out(m_acados_ocp_capsule);
    m_nlp_solver = mhe_autousv_ode_with_noisy_param_acados_get_nlp_solver(m_acados_ocp_capsule);
  }
}

const Config & NmheEstimator::get_config() const
{
  return m_config;
}

void NmheEstimator::set_config(const Config & config)
{
  m_config = config;
  // apply_config(m_config);
}

NmheEstimator::EnvEstimation NmheEstimator::compute_env_estimation(
  const std::vector<State> & states,
  const std::vector<MotorReport> & left_inputs,
  const std::vector<MotorReport> & right_inputs)
{
  if (m_measurements.empty()) {
    init_interpolate_measurements(states, left_inputs, right_inputs);
  } else {
    update_measurements(states, left_inputs, right_inputs);
  }
  apply_weights(m_config.optimization_param());
  set_start_measurements();
  set_measurements(Index{1}, Index{1}, N - 1);
  set_terminal_measurements(N);
  set_initial_guess();
  solve();
  std::array<AcadosReal, NX_AUGMENTED> ret{};
  ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, N, "x", ret.data());
  NmheEstimator::EnvEstimation res{rosidl_runtime_cpp::MessageInitialization::ALL};
  {
    using T = decltype(res.tau_env_u);
    using time_utils::to_message;
    res.stamp = to_message(m_last_estimate_time);
    res.tau_env_u = static_cast<T>(ret[3]);
    res.tau_env_v = static_cast<T>(ret[4]);
    res.tau_env_r = static_cast<T>(ret[5]);
    m_last_estimation = std::make_tuple(ret[0], ret[1], ret[2], res);
  }
  {
    // save initial guess for next estimation
    ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, 1, "x", ret.data());
    m_initial_guess = ret;
  }
  return res;
}

void NmheEstimator::init_interpolate_measurements(
  const std::vector<State> & states,
  const std::vector<MotorReport> & left_inputs,
  const std::vector<MotorReport> & right_inputs)
{
  using time_utils::from_message;
  const auto t1 = from_message(states.back().header.stamp);
  const auto t2 = from_message(left_inputs.back().stamp);
  const auto t3 = from_message(right_inputs.back().stamp);
  const auto estimate_time = std::min(std::min(t1, t2), t3);
  // update last estimate time point
  m_last_estimate_time = estimate_time;
  auto shooting_time = estimate_time;
  auto iter1 = states.cend() - 1;
  auto iter2 = left_inputs.cend() - 1;
  auto iter3 = right_inputs.cend() - 1;
  for (size_t i = N + 1; i > 0; --i) {
    shooting_time -= solver_time_step;
    while (from_message((*iter1).header.stamp) >= shooting_time && iter1 != states.cbegin()) {
      iter1 -= 1;
    }
    while (from_message((*iter2).stamp) >= shooting_time && iter2 != left_inputs.cbegin()) {
      iter2 -= 1;
    }
    while (from_message((*iter3).stamp) >= shooting_time && iter3 != right_inputs.cbegin()) {
      iter3 -= 1;
    }
    m_measurements.emplace_front(
      compute_interpolate_measurements(iter1, iter2, iter3, shooting_time));
  }
}

void NmheEstimator::solve()
{
  const auto solve_ret = mhe_autousv_ode_with_noisy_param_acados_solve(m_acados_ocp_capsule);
  if (0 != solve_ret) {
    std::string err_str{"Solver error: ", std::string::allocator_type{}};
    err_str += std::to_string(solve_ret);
    throw std::runtime_error{err_str};
  }
}

Measurements NmheEstimator::compute_interpolate_measurements(
  const NmheEstimator::iterT1 states_iter,
  const NmheEstimator::iterT2 left_inputs_iter,
  const NmheEstimator::iterT2 right_inputs_iter,
  const std::chrono::system_clock::time_point shooting_time)
{
  Measurements measurement{};
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using time_utils::from_message;
  {
    const auto t11 = from_message((*states_iter).header.stamp);
    const auto t12 = from_message((*(states_iter + 1)).header.stamp);
    const auto interpolated_point1 = duration_cast<duration<float>>(shooting_time - t11) /
                                     duration_cast<duration<float>>(t12 - t11);
    {
      const auto interpolated_u = motion_common::interpolate(
        (*states_iter).state.longitudinal_velocity_mps,
        (*(states_iter + 1)).state.longitudinal_velocity_mps,
        interpolated_point1);
      measurement.u = interpolated_u;
    }
    {
      const auto interpolated_v = motion_common::interpolate(
        (*states_iter).state.lateral_velocity_mps,
        (*(states_iter + 1)).state.lateral_velocity_mps,
        interpolated_point1);
      measurement.v = interpolated_v;
    }
    {
      const auto interpolated_r = motion_common::interpolate(
        (*states_iter).state.heading_rate_rps,
        (*(states_iter + 1)).state.heading_rate_rps,
        interpolated_point1);
      measurement.r = interpolated_r;
    }
  }
  // TODO convert power to force
  {
    const auto t21 = from_message((*left_inputs_iter).stamp);
    const auto t22 = from_message((*(left_inputs_iter + 1)).stamp);
    const auto interpolated_point2 = duration_cast<duration<float>>(shooting_time - t21) /
                                     duration_cast<duration<float>>(t22 - t21);
    auto f11 = static_cast<Real>((*left_inputs_iter).motor_power);
    if ((*left_inputs_iter).rotate_direction == MotorReport::BACKWARD) f11 = -f11;
    auto f12 = static_cast<Real>((*(left_inputs_iter + 1)).motor_power);
    if ((*(left_inputs_iter + 1)).rotate_direction == MotorReport::BACKWARD) f12 = -f12;
    const auto interpolated_f1 = motion_common::interpolate(f11, f12, interpolated_point2);
    measurement.left_input = interpolated_f1;
  }
  {
    const auto t31 = from_message((*right_inputs_iter).stamp);
    const auto t32 = from_message((*(right_inputs_iter + 1)).stamp);
    const auto interpolated_point3 = duration_cast<duration<float>>(shooting_time - t31) /
                                     duration_cast<duration<float>>(t32 - t31);
    auto f21 = static_cast<Real>((*right_inputs_iter).motor_power);
    if ((*right_inputs_iter).rotate_direction == MotorReport::BACKWARD) f21 = -f21;
    auto f22 = static_cast<Real>((*(right_inputs_iter + 1)).motor_power);
    if ((*(right_inputs_iter + 1)).rotate_direction == MotorReport::BACKWARD) f22 = -f22;
    const auto interpolated_f2 = motion_common::interpolate(f21, f22, interpolated_point3);
    measurement.right_input = interpolated_f2;
  }
  return measurement;
}

void NmheEstimator::update_measurements(
  const std::vector<State> & states,
  const std::vector<MotorReport> & left_inputs,
  const std::vector<MotorReport> & right_inputs)
{
  const auto shooting_time = m_last_estimate_time + solver_time_step;
  auto iter1 = states.cend() - 1;
  auto iter2 = left_inputs.cend() - 1;
  auto iter3 = right_inputs.cend() - 1;
  using time_utils::from_message;
  while (from_message((*iter1).header.stamp) >= shooting_time && iter1 != states.cbegin()) {
    iter1 -= 1;
  }
  while (from_message((*iter2).stamp) >= shooting_time && iter2 != left_inputs.cbegin()) {
    iter2 -= 1;
  }
  while (from_message((*iter3).stamp) >= shooting_time && iter3 != right_inputs.cbegin()) {
    iter3 -= 1;
  }
  m_measurements.emplace_back(compute_interpolate_measurements(iter1, iter2, iter3, shooting_time));
  m_measurements.pop_front();
  m_last_estimate_time = shooting_time;
}

Index NmheEstimator::get_compute_iterations() const
{
  Index sqp_iter{};
  ocp_nlp_get(m_nlp_config, m_nlp_solver, "sqp_iter", &sqp_iter);
  return sqp_iter;
}

}  // namespace nmhe_estimator
}  // namespace control
}  // namespace motion