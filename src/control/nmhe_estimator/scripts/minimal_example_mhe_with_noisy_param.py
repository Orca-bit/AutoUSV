import sys

from model import export_model
from export_mhe_ode_model_with_noisy_param import export_mhe_ode_model_with_noisy_param

from export_ocp_solver import export_ocp_solver
from export_mhe_solver_with_param import export_mhe_solver_with_param

import numpy as np
from scipy.linalg import block_diag

from utils import plot_sim


# general
Tf = 2.0
N = 40
h = Tf/N
Fmax = 250

p = np.array([172.0, 188.0, 24.0, 38.0, 168.0, 16.0, 0.8])

# NOTE: hard coded in export_model;
tau_env_true = np.array([30.0, 30.0, 10.0])

# ocp model and solver
model = export_model()

nx = model.x.size()[0]
nu = model.u.size()[0]

Q_ocp = 2*np.diag([1e3, 1e3, 1e2, 1e-1, 1e-1, 1e-2])
R_ocp = 1e-2 *np.eye(2)

acados_solver_ocp = export_ocp_solver(model, N, h, Q_ocp, R_ocp, Fmax)

# mhe model and solver
model_mhe = export_mhe_ode_model_with_noisy_param()

nx_augmented = model_mhe.x.size()[0]
nw = model_mhe.u.size()[0]
nx_mhe = nx_augmented - 3
ny = nx

Q0_mhe = np.diag([10, 10, 10, 7e-4, 7e-4, 1e-2])
Q_mhe  = np.diag([3.15, 8.15, 3.15, 5e-5, 5e-5, 1e-2])
R_mhe  = np.diag([0.045, 0.685, 0.035])

acados_solver_mhe = export_mhe_solver_with_param(model_mhe, N, h, Q_mhe, Q0_mhe, R_mhe)

# simulation
# v_stds = [0.2, 0.5, 1, 1]
# v_stds = [0, 0, 0, 0, 0, 0]
v_stds = [0, 0, 0, 0.2, 0.2, 0.2]

simX = np.ndarray((N+1, nx))
simU = np.ndarray((N, nu))
simY = np.ndarray((N+1, nx))

simXest = np.zeros((N+1, nx_mhe))
simWest = np.zeros((N, nx_augmented))
sim_env_est = np.zeros((N+1, 3))

# arrival cost mean (with wrong guess for l)
# x0_bar = np.array([0.0, np.pi, 0.0, 0.0, 1])
x0_bar = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# solve ocp problem
status = acados_solver_ocp.solve()
ocp_time = acados_solver_ocp.get_stats('time_tot')

if status != 0:
    raise Exception('acados returned status {}. Exiting.'.format(status))

# get solution
for i in range(N):
    simX[i,:] = acados_solver_ocp.get(i, "x")
    simU[i,:] = acados_solver_ocp.get(i, "u")
    simY[i,:] = simX[i,:] + np.transpose(np.diag(v_stds) @ (np.random.standard_normal((nx, 1))))

simX[N,:] = acados_solver_ocp.get(N, "x")
simY[N,:] = simX[N,:] + np.transpose(np.diag(v_stds) @ (np.random.standard_normal((nx, 1))))

# set measurements and controls
yref_0 = np.zeros((nx_mhe + 2*nx_augmented, ))
yref_0[:nx_mhe] = simY[0, 3:6]
yref_0[nx_mhe+nx_augmented:] = x0_bar
acados_solver_mhe.set(0, "yref", yref_0)
p0 = np.concatenate((simU[0,:], p), axis=None)
acados_solver_mhe.set(0, "p", p0)

# set initial guess to x0_bar
acados_solver_mhe.set(0, "x", x0_bar)

# yref = np.zeros((2*nx_mhe, ))
for j in range(1, N):
    yref = np.zeros((nx_mhe+nx_augmented, ))
    yref[:nx_mhe] = simY[j, 3:6]
    acados_solver_mhe.set(j, "yref", yref)
    pp = np.concatenate((simU[j,:], p), axis=None)
    acados_solver_mhe.set(j, "p", pp)

    # set initial guess to x0_bar
    acados_solver_mhe.set(j, "x", x0_bar)

# set initial guess to x0_bar
acados_solver_mhe.set(N, "x", x0_bar)

# solve mhe problem
status = acados_solver_mhe.solve()
mhe_time = acados_solver_mhe.get_stats('time_tot')

if status != 0:
    raise Exception('acados returned status {}. Exiting.'.format(status))

# get solution
for i in range(N):
    x_augmented = acados_solver_mhe.get(i, "x")

    simXest[i,:] = x_augmented[:nx_mhe]
    sim_env_est[i,:] = x_augmented[nx_mhe:]
    simWest[i,:] = acados_solver_mhe.get(i, "u")
    

x_augmented = acados_solver_mhe.get(N, "x")
simXest[N,:] = x_augmented[:nx_mhe]
sim_env_est[N,:] = x_augmented[nx_mhe:]

print('ocp solve time ', ocp_time)
print('mhe solve time ', mhe_time)
print('difference |x0_est - x0_bar|', np.linalg.norm(x0_bar[0:nx_mhe] - simXest[0, :]))
print('difference |x_est - x_true|', np.linalg.norm(simXest - simX[:, 3:6]))
print(sim_env_est[-1])
print('difference |env_est - env_true|', np.linalg.norm(sim_env_est[-1] - tau_env_true))

ts = np.linspace(0, Tf, N+1)
plot_sim(ts, Fmax, simU, simX, simXest, simY, latexify=False)


import matplotlib.pyplot as plt
import os

env_labels = ['env_u', 'env_v', 'env_r']
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(ts, tau_env_true[i]*np.ones((N+1, 1)), '-', label='true')
    plt.plot(ts, sim_env_est[:,i], '.-', label='estimated')
    plt.grid()
    plt.ylabel(env_labels[i])
    plt.xlabel('$t$')
    # plt.legend(['true l', 'estimated l'])
    plt.legend(loc=1)

plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)

# avoid plotting when running on Travis
if os.environ.get('ACADOS_ON_TRAVIS') is None:
    plt.show()
