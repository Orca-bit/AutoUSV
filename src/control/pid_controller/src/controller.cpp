//
// Created by liuhao on 2022/4/10.
//

#include "pid_controller/controller.hpp"

double dead_zone_force(double force, double pos_limit, double neg_limit)
{
  if (force > 0) {
    if (force < pos_limit) {
      return 0;
    }  // if
  } else {
    if (force > -neg_limit) {
      return 0;
    }  // if
  }    // else

  return force;
}

Controller::Controller(PidGain pid, Limits limits) : fvel_pid_gain_(pid), limits_(limits)
{
  fvel_pid_.reset();
  fvel_pid_.setGains(
    fvel_pid_gain_.kp_,
    fvel_pid_gain_.ki_,
    fvel_pid_gain_.kd_,
    fvel_pid_gain_.imax_,
    fvel_pid_gain_.imin_);
}

double Controller::compute_fwd_force(double fvel_cmd, double fvel_meas)
{
  // calculate pid force X
  double fvel_error = fvel_cmd - fvel_meas;
  double fvel_comp_output = fvel_pid_.computeCommand(
    fvel_error, std::chrono::nanoseconds{std::chrono::seconds{1 / 20}}.count());
  fvel_comp_output = fvel_comp_output + fvel_pid_gain_.kf_ * fvel_cmd;

  return dead_zone_force(
    fvel_comp_output, limits_.max_fwd_force_ * 0.06, limits_.max_bck_force_ * 0.06);
}
