import sys
sys.path.insert(0, 'common')

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from model import export_model
from utils import plot_sim
import numpy as np
import scipy.linalg

# create ocp object to formulate the OCP
ocp = AcadosOcp()

# set model
model = export_model()
ocp.model = model

#set parameters
params = np.array([172, 188, 24, 38, 168, 16, 0.8, 0, 0, 0])
ocp.parameter_values = params

Tf = 2.0
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx
N = 40

# set dimensions
ocp.dims.N = N
# NOTE: all dimensions but N are now detected automatically in the Python
#  interface, all other dimensions will be overwritten by the detection.

# set cost module
ocp.cost.cost_type = 'LINEAR_LS'
ocp.cost.cost_type_e = 'LINEAR_LS'

Q = 2*np.diag([1e3, 1e3, 1e2, 1e-1, 1e-1, 1e-2])
R = 2*np.diag([1e-2, 1e-2])

ocp.cost.W = scipy.linalg.block_diag(Q, R)

ocp.cost.W_e = Q

ocp.cost.Vx = np.zeros((ny, nx))
ocp.cost.Vx[:nx,:nx] = np.eye(nx)

Vu = np.zeros((ny, nu))
for i in range(2):
    Vu[nx+i,i] = 1.0
ocp.cost.Vu = Vu

ocp.cost.Vx_e = np.eye(nx)

ocp.cost.yref  = np.zeros((ny, ))
ocp.cost.yref_e = np.zeros((ny_e, ))

# set constraints
Fmax = 250
x0 = np.array([5.0, 3.0, np.pi, 0.0, 0.0, 0.0])
ocp.constraints.constr_type = 'BGH'
ocp.constraints.lbu = np.array([-Fmax, -Fmax])
ocp.constraints.ubu = np.array([+Fmax, +Fmax])
ocp.constraints.x0 = x0
ocp.constraints.idxbu = np.array([0, 1])

ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI

ocp.solver_options.qp_solver_cond_N = N

# set prediction horizon
ocp.solver_options.tf = Tf

acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + model.name + '.json')
acados_integrator = AcadosSimSolver(ocp, json_file = 'acados_ocp_' + model.name + '.json')

Nsim = 100
simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim, nu))

xcurrent = x0
simX[0,:] = xcurrent

# print(ocp.parameter_values)

# closed loop
for i in range(Nsim):

    # solve ocp
    acados_ocp_solver.set(0, "lbx", xcurrent)
    acados_ocp_solver.set(0, "ubx", xcurrent)

    status = acados_ocp_solver.solve()

    if status != 0:
        raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))

    simU[i,:] = acados_ocp_solver.get(0, "u")

    # simulate system
    acados_integrator.set("x", xcurrent)
    acados_integrator.set("u", simU[i,:])

    status = acados_integrator.solve()
    if status != 0:
        raise Exception('acados integrator returned status {}. Exiting.'.format(status))

    # update state
    xcurrent = acados_integrator.get("x")
    simX[i+1,:] = xcurrent

# plot results
plot_sim(np.linspace(0, Tf/N*Nsim, Nsim+1), Fmax, simU, simX)
