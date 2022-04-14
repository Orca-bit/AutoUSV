import numpy as np
import scipy
from acados_template import *

def export_ocp_solver(model, N, h, Q, R, Fmax=250):

    ocp = AcadosOcp()

    # set model
    ocp.model = model

    Tf = N*h
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.dims.N = N

    # set cost
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ocp.cost.W = scipy.linalg.block_diag(R, Q)

    ocp.cost.W_e = Q

    x = ocp.model.x
    u = ocp.model.u

    ocp.model.cost_y_expr = vertcat(u, x)

    ocp.model.cost_y_expr_e = x

    ocp.cost.yref  = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((ny_e, ))

    # setting bounds
    ocp.constraints.lbu = np.array([-Fmax, -Fmax])
    ocp.constraints.ubu = np.array([+Fmax, +Fmax])
    ocp.constraints.x0 = np.array([0.0, 0.0, np.pi, 2.0, 0.0, 0.0])
    ocp.constraints.idxbu = np.array([0, 1])

    # set QP solver
    # ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'

    # set prediction horizon
    ocp.solver_options.tf = Tf
    ocp.solver_options.nlp_solver_type = 'SQP'
    # ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.nlp_solver_max_iter = 500

    acados_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    return acados_solver
