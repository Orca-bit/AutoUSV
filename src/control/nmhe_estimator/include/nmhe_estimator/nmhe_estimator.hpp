//
// Created by liuhao on 2022/1/11.
//

#ifndef NMHE_ESTIMATOR_NMHE_ESTIMATOR_HPP
#define NMHE_ESTIMATOR_NMHE_ESTIMATOR_HPP

// acados
#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
#include <acados_solver_mhe_autousv_ode_with_noisy_param.h>

#include <deque>
#include <experimental/optional>

#include <nmhe_estimator/config.hpp>
#include <nmhe_estimator/visibility_control.hpp>

#include <usv_msgs/msg/env_estimation.hpp>
#include <usv_msgs/msg/motor_report1.hpp>
#include <usv_msgs/msg/vehicle_kinematic_state.hpp>

namespace motion
{
namespace control
{
namespace nmhe_estimator
{
constexpr size_t N = MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_N;
constexpr size_t NX_AUGMENTED = MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NX;
constexpr size_t NW = MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NU;
constexpr size_t NY = MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NY;
constexpr size_t NY0 = MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NY0;
constexpr size_t NYN = MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NYN;
constexpr size_t NP = MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NP;

struct NMHE_ESTIMATOR_LOCAL Measurements
{
  Real u{0.};            // longitudinal velocity
  Real v{0.};            // lateral velocity
  Real r{0.};            // yaw rate
  Real left_input{0.};   // left motor force
  Real right_input{0.};  // right motor force
};

class NMHE_ESTIMATOR_PUBLIC NmheEstimator
{
public:
  // TODO need adjustment
  constexpr static std::chrono::nanoseconds solver_time_step{std::chrono::milliseconds{100LL}};

  NmheEstimator(const NmheEstimator &) = delete;
  NmheEstimator(NmheEstimator &&) = delete;
  NmheEstimator & operator=(const NmheEstimator &) = delete;
  NmheEstimator & operator=(NmheEstimator &&) = delete;

  using State = usv_msgs::msg::VehicleKinematicState;
  using EnvEstimation = usv_msgs::msg::EnvEstimation;
  using MotorReport = usv_msgs::msg::MotorReport1;

  explicit NmheEstimator(Config config);
  ~NmheEstimator();

  const Config & get_config() const;
  void set_config(const Config & config);

  Index get_compute_iterations() const;

  EnvEstimation compute_env_estimation(
    const std::vector<State> & states,
    const std::vector<MotorReport> & left_inputs,
    const std::vector<MotorReport> & right_inputs);

private:
  NMHE_ESTIMATOR_LOCAL void acados_init();
  NMHE_ESTIMATOR_LOCAL void solve();
  // NMHE_ESTIMATOR_LOCAL void set_measurements_and_parameters();
  NMHE_ESTIMATOR_LOCAL void init_interpolate_measurements(
    const std::vector<State> & states,
    const std::vector<MotorReport> & left_inputs,
    const std::vector<MotorReport> & right_inputs);
  NMHE_ESTIMATOR_LOCAL void update_measurements(
    const std::vector<State> & states,
    const std::vector<MotorReport> & left_inputs,
    const std::vector<MotorReport> & right_inputs);
  using iterT1 = std::vector<State>::const_iterator;
  using iterT2 = std::vector<MotorReport>::const_iterator;
  NMHE_ESTIMATOR_LOCAL Measurements compute_interpolate_measurements(
    iterT1 states_iter,
    iterT2 left_inputs_iter,
    iterT2 right_inputs_iter,
    std::chrono::system_clock::time_point shooting_time);
  // NMHE_ESTIMATOR_LOCAL void apply_config(const Config & config);
  // NMHE_ESTIMATOR_LOCAL void apply_parameters(const VehicleConfig & cfg) noexcept;
  NMHE_ESTIMATOR_LOCAL void apply_weights(const OptimizationConfig & cfg);
  NMHE_ESTIMATOR_LOCAL void set_start_weights(
    const Weights & r, const Weights & q, const Weights & q0);
  NMHE_ESTIMATOR_LOCAL void set_terminal_weights(const Weights & r);
  NMHE_ESTIMATOR_LOCAL void apply_nominal_weights(
    const Weights & r, const Weights & q, Index start, Index end);
  NMHE_ESTIMATOR_LOCAL void set_measurements(Index y_start, Index measurements_start, Index count);
  NMHE_ESTIMATOR_LOCAL void set_start_measurements();
  NMHE_ESTIMATOR_LOCAL void set_terminal_measurements(Index measurement_idx);
  NMHE_ESTIMATOR_LOCAL void set_initial_guess();

  Config m_config;
  std::deque<Measurements> m_measurements{};
  std::chrono::system_clock::time_point m_last_estimate_time{};
  // EnvEstimation m_last_estimation{};
  // last estimation: u v r env
  std::experimental::optional<std::tuple<Real, Real, Real, EnvEstimation>> m_last_estimation{};
  // for init guess
  std::array<double, NX_AUGMENTED> m_initial_guess{};
  // acados
  mhe_autousv_ode_with_noisy_param_solver_capsule * m_acados_ocp_capsule;
  ocp_nlp_config * m_nlp_config{nullptr};
  ocp_nlp_dims * m_nlp_dims{nullptr};
  ocp_nlp_in * m_nlp_in{nullptr};
  ocp_nlp_out * m_nlp_out{nullptr};
  ocp_nlp_solver * m_nlp_solver{nullptr};
};

}  // namespace nmhe_estimator
}  // namespace control
}  // namespace motion

#endif  // NMHE_ESTIMATOR_NMHE_ESTIMATOR_HPP
