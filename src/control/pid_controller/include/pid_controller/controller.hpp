//
// Created by liuhao on 2022/4/10.
//

#ifndef PID_CONTROLLER_CONTROLLER_HPP
#define PID_CONTROLLER_CONTROLLER_HPP

#include <control_toolbox/pid.hpp>

struct PidGain
{
  double kf_;
  double kp_;
  double ki_;
  double kd_;
  double imax_;
  double imin_;
};

struct Limits
{
  double max_fwd_vel_;
  double max_fwd_force_;
  double max_bck_vel_;
  double max_bck_force_;
};

class Controller
{
public:
  Controller(PidGain pid, Limits limits);
  ~Controller() = default;

  // if only fwd control, just need this result divided by 2
  double compute_fwd_force(double fvel_cmd, double fvel_meas);

private:
  control_toolbox::Pid fvel_pid_;
  PidGain fvel_pid_gain_;
  Limits limits_;
};

#endif  // PID_CONTROLLER_CONTROLLER_HPP
