//
// Created by liuhao on 2021/11/27.
//

#ifndef MOTION_COMMON_USV_DYNAMICS_SNIPPET_HPP
#define MOTION_COMMON_USV_DYNAMICS_SNIPPET_HPP

DifferentialState x, y, psi;  // pose
DifferentialState u, v, r;  // velocities
Control left_cmd, right_cmd;

// vehicle parameters
OnlineData m1, m2, m3; // mass matrix
OnlineData d1, d2, d3; // damping matrix
OnlineData L; // lateral distance from cg to thrusters
OnlineData tau_env_u, tau_env_v, tau_env_r; // environment forces

IntermediateState f_u = (m2 * v * r - d1 * u) / m1;
IntermediateState f_v = (- m1 * u * r - d2 * v) / m2;
IntermediateState f_r = ((m1 - m2) * u * v - d3 * r) / m3;

DifferentialEquation f;

f << dot(x) == ((u * cos(psi)) - (v * sin(psi)));
f << dot(y) == ((u * sin(psi)) + (v * cos(psi)));
f << dot(psi) == r;
f << dot(u) == (left_cmd + right_cmd + tau_env_u) / m1 + f_u;
f << dot(v) == tau_env_v / m2 + f_v;
f << dot(r) == ((right_cmd - left_cmd) * L + tau_env_r) / m3 + f_r;

#endif  // MOTION_COMMON_USV_DYNAMICS_SNIPPET_HPP
