from acados_template import *

def export_mhe_ode_model_with_noisy_param():
    '''
    Export ode model augmented with an additional state corresponding to the 
    parameters, which are identified online
    '''

    model_name = 'mhe_autousv_ode_with_noisy_param'

    # constants
    # m1 = 172.0
    # m2 = 188.0
    # m3 = 24.0
    # d1 = 38.0
    # d2 = 168.0
    # d3 = 16.0
    # l = 0.8
    m1 = SX.sym('m1')
    m2 = SX.sym('m2') 
    m3 = SX.sym('m3') 
    d1 = SX.sym('d1')
    d2 = SX.sym('d2') 
    d3 = SX.sym('d3') 
    l = SX.sym('l')
    #tau_env_u = 10.0
    #tau_env_v = 10.0
    #tau_env_r = 10.0 -> now estimated


    # set up states
    u = SX.sym('u')
    v = SX.sym('v')
    r = SX.sym('r')
    # add parameters as states
    tau_env_u = SX.sym('env_u')
    tau_env_v = SX.sym('env_v')
    tau_env_r = SX.sym('env_r')

    x = vertcat(u, v, r, tau_env_u, tau_env_v, tau_env_r)

    # state noise
    w_u = SX.sym('w_u')
    w_v = SX.sym('w_v')
    w_r = SX.sym('w_r')
    w_env_u = SX.sym('w_env_u')
    w_env_v = SX.sym('w_env_v')
    w_env_r = SX.sym('w_env_r')

    w = vertcat(w_u, w_v, w_r, w_env_u, w_env_v, w_env_r)

    # xdot
    u_dot = SX.sym('u_dot')
    v_dot = SX.sym('v_dot')
    r_dot = SX.sym('r_dot')
    tau_env_u_dot = SX.sym('env_u_dot')
    tau_env_v_dot = SX.sym('env_v_dot')
    tau_env_r_dot = SX.sym('env_r_dot')

    xdot = vertcat(u_dot, v_dot, r_dot, tau_env_u_dot, tau_env_v_dot, tau_env_r_dot)

    # algebraic variables
    z = []

    # parameters <= controls
    left_ctrl = SX.sym('left_ctrl')
    right_ctrl = SX.sym('right_ctrl')
    p = vertcat(left_ctrl, right_ctrl, m1, m2, m3, d1, d2, d3, l)

    # dynamics
    du = (left_ctrl + right_ctrl + tau_env_u + m2 * v * r - d1 * u) / m1
    dv = (tau_env_v - m1 * u * r - d2 * v) / m2
    dr = ((right_ctrl - left_ctrl) * l + tau_env_r + (m1 - m2) * u * v - d3 * r) / m3

    f_expl = vertcat(du, dv, dr, 0, 0, 0)

    # add additive state noise
    f_expl = f_expl + w
    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = w
    model.z = z
    model.p = p
    model.name = model_name

    return model

