import numpy as np
from scipy.linalg import block_diag
from acados_template import *


def export_mhe_solver_with_param(model, N, h, Q, Q0, R):

    # create render arguments
    ocp_mhe = AcadosOcp()

    ocp_mhe.model = model

    nx_augmented = model.x.size()[0]
    nu = model.u.size()[0]
    nparam = model.p.size()[0]
    nx = nx_augmented-3

    ny = R.shape[0] + Q.shape[0]                    # h(x), w 
    ny_e = R.shape[0]
    ny_0 = R.shape[0] + Q.shape[0] + Q0.shape[0]    # h(x), w and arrival cost

    # set number of shooting nodes
    ocp_mhe.dims.N = N

    x = ocp_mhe.model.x
    u = ocp_mhe.model.u

    # set cost type
    ocp_mhe.cost.cost_type = 'NONLINEAR_LS'
    ocp_mhe.cost.cost_type_e = 'NONLINEAR_LS'
    ocp_mhe.cost.cost_type_0 = 'NONLINEAR_LS' 

    ocp_mhe.cost.W_0 = block_diag(R, Q, Q0)
    ocp_mhe.model.cost_y_expr_0 = vertcat(x[:nx], u, x)
    ocp_mhe.cost.yref_0 = np.zeros((ny_0,))

    # cost intermediate stages
    ocp_mhe.cost.W = block_diag(R, Q)

    ocp_mhe.model.cost_y_expr = vertcat(x[:nx], u)
    
    ocp_mhe.cost.W_e = R
    ocp_mhe.model.cost_y_expr_e = x[:nx]

    ocp_mhe.parameter_values = np.zeros((nparam, ))

    # set y_ref for all stages
    ocp_mhe.cost.yref  = np.zeros((ny,))
    ocp_mhe.cost.yref_e = np.zeros((ny_e, ))
    ocp_mhe.cost.yref_0  = np.zeros((ny_0,))

    # set QP solver
    ocp_mhe.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    # ocp_mhe.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp_mhe.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp_mhe.solver_options.integrator_type = 'ERK'

    # set prediction horizon
    ocp_mhe.solver_options.tf = N*h

    ocp_mhe.solver_options.nlp_solver_type = 'SQP'
    # ocp_mhe.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp_mhe.solver_options.nlp_solver_max_iter = 1000

    acados_solver_mhe = AcadosOcpSolver(ocp_mhe, json_file = 'acados_ocp.json')

    # set arrival cost weighting matrix
    # acados_solver_mhe.cost_set(0, "W", block_diag(R, Q, Q0))

    return acados_solver_mhe
