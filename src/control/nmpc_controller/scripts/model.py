from acados_template import *
from casadi import SX, vertcat, sin, cos

def export_model():

	model_name = 'autousv'

	#parameters
	m1 = SX.sym('m1')
	m2 = SX.sym('m2')
	m3 = SX.sym('m3')
	d1 = SX.sym('d1')
	d2 = SX.sym('d2')
	d3 = SX.sym('d3')
	l = SX.sym('l')
	tau_env_u = SX.sym('tau_env_u')
	tau_env_v = SX.sym('tau_env_v')
	tau_env_r = SX.sym('tau_env_r')
	params = vertcat(m1, m2, m3, d1, d2, d3, l, tau_env_u, tau_env_v, tau_env_r)

	# m1 = 172.0
	# m2 = 188.0
	# m3 = 24.0
	# d1 = 38.0
	# d2 = 168.0
	# d3 = 16.0
	# l = 0.8
	# tau_env_u = 0.0
	# tau_env_v = 0.0
	# tau_env_r = 0.0

	#states
	x = SX.sym('x')
	y = SX.sym('y')
	psi = SX.sym('psi')
	u = SX.sym('u')
	v = SX.sym('v')
	r = SX.sym('r')
	states = vertcat(x, y, psi, u, v, r) 

	#controls
	left_ctrl = SX.sym('left_ctrl')
	right_ctrl = SX.sym('right_ctrl')
	controls = vertcat(left_ctrl, right_ctrl)

	#model equations
	dx = u * cos(psi) - v * sin(psi)
	dy = u * sin(psi) + v * cos(psi)
	dpsi = r
	du = (left_ctrl + right_ctrl + tau_env_u + m2 * v * r - d1 * u) / m1
	dv = (tau_env_v - m1 * u * r - d2 * v) / m2
	dr = ((right_ctrl - left_ctrl) * l + tau_env_r + (m1 - m2) * u * v - d3 * r) / m3

	f_expl = vertcat(dx, dy, dpsi, du, dv, dr)

	dot_x = SX.sym('dot_x')
	dot_y = SX.sym('dot_y')
	dot_psi = SX.sym('dot_psi')
	dot_u = SX.sym('dot_u')
	dot_v = SX.sym('dot_v')
	dot_r = SX.sym('dot_r')
	dot_states = vertcat(dot_x, dot_y, dot_psi, dot_u, dot_v, dot_r)

	f_impl = dot_states - f_expl

	model = AcadosModel()

	model.f_expl_expr = f_expl
	model.f_impl_expr = f_impl
	model.x = states
	model.xdot = dot_states
	model.u = controls
	model.p = params
	model.name = model_name

	return model
