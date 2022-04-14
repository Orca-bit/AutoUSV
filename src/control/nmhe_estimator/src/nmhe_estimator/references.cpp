//
// Created by liuhao on 2022/1/13.
//

#include <nmhe_estimator/nmhe_estimator.hpp>

namespace motion
{
namespace control
{
namespace nmhe_estimator
{
using AcadosReal = real_t;

constexpr auto HORIZON = N;

void NmheEstimator::apply_weights(const OptimizationConfig & cfg)
{
  set_start_weights(cfg.r(), cfg.q(), cfg.q0());
  apply_nominal_weights(cfg.r(), cfg.q(), Index{1}, HORIZON);
  set_terminal_weights(cfg.r());
}

void NmheEstimator::set_start_weights(const Weights & r, const Weights & q, const Weights & q0)
{
  static_assert(NY0 == 15, "Unexpected number of start reference variables");
  std::array<AcadosReal, NY0 * NY0> W0{};
  W0[0] = r.u_weight();
  W0[NY0 + 1] = r.v_weight();
  W0[NY0 * 2 + 2] = r.r_weight();
  W0[NY0 * 3 + 3] = q.u_noise_weight();
  W0[NY0 * 4 + 4] = q.v_noise_weight();
  W0[NY0 * 5 + 5] = q.r_noise_weight();
  W0[NY0 * 6 + 6] = q.env_u_noise_weight();
  W0[NY0 * 7 + 7] = q.env_v_noise_weight();
  W0[NY0 * 8 + 8] = q.env_r_noise_weight();
  W0[NY0 * 9 + 9] = q0.u_weight();
  W0[NY0 * 10 + 10] = q0.v_weight();
  W0[NY0 * 11 + 11] = q0.r_weight();
  W0[NY0 * 12 + 12] = q0.env_u_weight();
  W0[NY0 * 13 + 13] = q0.env_v_weight();
  W0[NY0 * 14 + 14] = q0.env_r_weight();
  ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, 0, "W", W0.data());
}

void NmheEstimator::set_terminal_weights(const Weights & r)
{
  static_assert(NYN == 3, "Unexpected number of terminal reference variables");
  std::array<AcadosReal, NYN * NYN> WN{};
  WN[0] = r.u_weight();
  WN[NYN + 1] = r.v_weight();
  WN[NYN * 2 + 2] = r.r_weight();
  ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, N, "W", WN.data());
}

void NmheEstimator::apply_nominal_weights(
  const Weights & r, const Weights & q, const Index start, Index end)
{
  static_assert(sizeof(std::size_t) >= sizeof(Index), "static cast might truncate");
  end = std::min(static_cast<std::size_t>(end), HORIZON);
  if (start > end) {
    throw std::logic_error{"Inconsistent bounds: apply! There's likely an indexing bug somewhere"};
  }
  static_assert(NY == 9, "Unexpected number of reference variables");
  std::array<AcadosReal, NY * NY> W{};
  W[0] = r.u_weight();
  W[NY + 1] = r.v_weight();
  W[NY * 2 + 2] = r.r_weight();
  W[NY * 3 + 3] = q.u_noise_weight();
  W[NY * 4 + 4] = q.v_noise_weight();
  W[NY * 5 + 5] = q.r_noise_weight();
  W[NY * 6 + 6] = q.env_u_noise_weight();
  W[NY * 7 + 7] = q.env_v_noise_weight();
  W[NY * 8 + 8] = q.env_r_noise_weight();
  for (Index i = start; i < end; ++i) {
    ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, i, "W", W.data());
  }
}

void NmheEstimator::set_measurements(Index y_start, Index measurements_start, Index count)
{
  if ((y_start + count) > HORIZON || (measurements_start + count) > m_measurements.size()) {
    throw std::logic_error{"set_measurements would go out of bounds! Indexing bug likely!"};
  }
  std::array<AcadosReal, NY> yref{};
  std::array<AcadosReal, NP> params{
    0.,
    0.,
    m_config.vehicle_param().mass11(),
    m_config.vehicle_param().mass22(),
    m_config.vehicle_param().mass33(),
    m_config.vehicle_param().damping_11(),
    m_config.vehicle_param().damping_22(),
    m_config.vehicle_param().damping_33(),
    m_config.vehicle_param().length_cg_thrusters_lateral_m()};
  for (auto i = Index{}; i < count; ++i) {
    const auto & measurement = m_measurements[measurements_start + i];
    const auto ydx = y_start + i;
    yref[0] = static_cast<AcadosReal>(measurement.u);
    yref[1] = static_cast<AcadosReal>(measurement.v);
    yref[2] = static_cast<AcadosReal>(measurement.r);
    params[0] = static_cast<AcadosReal>(measurement.left_input);
    params[1] = static_cast<AcadosReal>(measurement.right_input);
    ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, ydx, "yref", yref.data());
    mhe_autousv_ode_with_noisy_param_acados_update_params(
      m_acados_ocp_capsule, ydx, params.data(), NP);
  }
}

void NmheEstimator::set_start_measurements()
{
  if (!m_measurements.empty()) {
    throw std::logic_error{"No measurement received!"};
  }
  {
    std::array<AcadosReal, NY0> yref_0{};
    yref_0[0] = m_measurements[0].u;
    yref_0[1] = m_measurements[0].v;
    yref_0[2] = m_measurements[0].r;
    yref_0[9] = m_initial_guess[0];
    yref_0[10] = m_initial_guess[1];
    yref_0[11] = m_initial_guess[2];
    yref_0[12] = m_initial_guess[3];
    yref_0[13] = m_initial_guess[4];
    yref_0[14] = m_initial_guess[5];
    ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, 0, "yref", yref_0.data());
  }
  {
    std::array<AcadosReal, NP> params_0{
      0.,
      0.,
      m_config.vehicle_param().mass11(),
      m_config.vehicle_param().mass22(),
      m_config.vehicle_param().mass33(),
      m_config.vehicle_param().damping_11(),
      m_config.vehicle_param().damping_22(),
      m_config.vehicle_param().damping_33(),
      m_config.vehicle_param().length_cg_thrusters_lateral_m()};
    params_0[0] = m_measurements[0].left_input;
    params_0[1] = m_measurements[0].right_input;
    mhe_autousv_ode_with_noisy_param_acados_update_params(
      m_acados_ocp_capsule, 0, params_0.data(), NP);
  }
}

void NmheEstimator::set_terminal_measurements(Index measurement_idx)
{
  if (measurement_idx >= m_measurements.size()) {
    throw std::logic_error{"set_terminal_measurements: no such idx!"};
  }
  std::array<AcadosReal, NYN> yref_n{};
  yref_n[0] = m_measurements[measurement_idx].u;
  yref_n[1] = m_measurements[measurement_idx].v;
  yref_n[2] = m_measurements[measurement_idx].r;
  ocp_nlp_cost_model_set(m_nlp_config, m_nlp_dims, m_nlp_in, N, "yref", yref_n.data());
}

void NmheEstimator::set_initial_guess()
{
  std::array<AcadosReal, NW> initial_w{};
  std::array<AcadosReal, NX_AUGMENTED> initial_x{
    m_measurements[0].u, m_measurements[0].v, m_measurements[0].r, 0, 0, 0};
  if (!m_last_estimation) {
    for (auto i = Index{}; i < N; ++i) {
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, i, "x", initial_x.data());
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, i, "u", initial_w.data());
    }
    ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, N, "x", initial_x.data());
  } else {
    for (auto i = Index{}; i < N; ++i) {
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, i, "u", initial_w.data());
      ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, i + 1, "x", initial_x.data());
      ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, i, "x", initial_x.data());
    }
    ocp_nlp_out_set(m_nlp_config, m_nlp_dims, m_nlp_out, N, "x", initial_x.data());
  }
}

}  // namespace nmhe_estimator
}  // namespace control
}  // namespace motion
