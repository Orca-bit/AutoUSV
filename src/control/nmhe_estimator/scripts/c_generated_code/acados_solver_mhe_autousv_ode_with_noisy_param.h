/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_mhe_autousv_ode_with_noisy_param_H_
#define ACADOS_SOLVER_mhe_autousv_ode_with_noisy_param_H_

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NX     6
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NZ     0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NU     6
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NP     9
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NBX    0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NBX0   0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NBU    0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSBX   0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSBU   0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSH    0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSG    0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSPHI  0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSHN   0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSGN   0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSPHIN 0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSBXN  0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NS     0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NSN    0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NG     0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NBXN   0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NGN    0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NY0    15
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NY     9
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NYN    3
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_N      40
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NH     0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NPHI   0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NHN    0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NPHIN  0
#define MHE_AUTOUSV_ODE_WITH_NOISY_PARAM_NR     0

#ifdef __cplusplus
extern "C" {
#endif

// ** capsule for solver data **
typedef struct mhe_autousv_ode_with_noisy_param_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *forw_vde_casadi;
    external_function_param_casadi *expl_ode_fun;




    // cost

    external_function_param_casadi *cost_y_fun;
    external_function_param_casadi *cost_y_fun_jac_ut_xt;
    external_function_param_casadi *cost_y_hess;


    external_function_param_casadi cost_y_0_fun;
    external_function_param_casadi cost_y_0_fun_jac_ut_xt;
    external_function_param_casadi cost_y_0_hess;



    external_function_param_casadi cost_y_e_fun;
    external_function_param_casadi cost_y_e_fun_jac_ut_xt;
    external_function_param_casadi cost_y_e_hess;


    // constraints




} mhe_autousv_ode_with_noisy_param_solver_capsule;

mhe_autousv_ode_with_noisy_param_solver_capsule * mhe_autousv_ode_with_noisy_param_acados_create_capsule(void);
int mhe_autousv_ode_with_noisy_param_acados_free_capsule(mhe_autousv_ode_with_noisy_param_solver_capsule *capsule);

int mhe_autousv_ode_with_noisy_param_acados_create(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
/**
 * Generic version of mhe_autousv_ode_with_noisy_param_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
int mhe_autousv_ode_with_noisy_param_acados_create_with_discretization(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
int mhe_autousv_ode_with_noisy_param_acados_update_time_steps(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule, int N, double* new_time_steps);
int mhe_autousv_ode_with_noisy_param_acados_update_params(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule, int stage, double *value, int np);
int mhe_autousv_ode_with_noisy_param_acados_solve(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
int mhe_autousv_ode_with_noisy_param_acados_free(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
void mhe_autousv_ode_with_noisy_param_acados_print_stats(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);

ocp_nlp_in *mhe_autousv_ode_with_noisy_param_acados_get_nlp_in(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
ocp_nlp_out *mhe_autousv_ode_with_noisy_param_acados_get_nlp_out(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
ocp_nlp_solver *mhe_autousv_ode_with_noisy_param_acados_get_nlp_solver(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
ocp_nlp_config *mhe_autousv_ode_with_noisy_param_acados_get_nlp_config(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
void *mhe_autousv_ode_with_noisy_param_acados_get_nlp_opts(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
ocp_nlp_dims *mhe_autousv_ode_with_noisy_param_acados_get_nlp_dims(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);
ocp_nlp_plan *mhe_autousv_ode_with_noisy_param_acados_get_nlp_plan(mhe_autousv_ode_with_noisy_param_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_mhe_autousv_ode_with_noisy_param_H_
