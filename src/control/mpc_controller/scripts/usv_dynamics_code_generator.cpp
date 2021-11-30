//
// Created by liuhao on 2021/11/27.
//

#include <acado/acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main(int argc, char ** argv)
{
#include <usv_dynamics.snippet.hpp>
  //
  // Weighting matrices and reference functions: not pre-specified
  //

  Function rf;
  Function rfN;

  rf << x << y << psi << u << v << r << left_cmd << right_cmd;
  rfN << x << y << psi << u << v << r;

  BMatrix W = eye<bool>(rf.getDim());
  BMatrix WN = eye<bool>(rfN.getDim());

  // 4 second time horizon
  const int N = 40;       // Number of steps
  const int Ni = 4;       // number of integrators
  const double Ts = 0.1;  // step length [second]

  OCP ocp(0, N * Ts, N);

  ocp.subjectTo(f);

  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);

  ocp.subjectTo(-2.0 <= u <= 3.0);              // longitude velocity limit [m/s]
  ocp.subjectTo(-1.0 <= v <= 1.0);              // transverse velocity limit [m/s]
  ocp.subjectTo(-0.611 <= r <= 0.611);          // angular velocity limit 35 degree [rad/s]
  ocp.subjectTo(-200.0 <= left_cmd <= 300.0);   // left thruster limit [N]
  ocp.subjectTo(-200.0 <= right_cmd <= 300.0);  // right thruster limit [N]

  //
  // Export the code:
  //
  OCPexport mpc(ocp);

  // See https://github.com/cho3/acado/blob/master/acado/utils/acado_types.hpp#L331..L427
  // for all options

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

  mpc.set(INTEGRATOR_TYPE, INT_RK45);  // explicit RK45
  mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  // mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(MAX_NUM_QP_ITERATIONS, 999);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  // mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
  // mpc.set(QP_SOLVER, QP_FORCES);

  mpc.set(LEVENBERG_MARQUARDT, 1.0E2);

  mpc.set(GENERATE_TEST_FILE, NO);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);

  // mpc.set(USE_SINGLE_PRECISION, YES);
  // mpc.set(CG_USE_OPENMP, YES);

  if (mpc.exportCode("single_track_dynamics") != SUCCESSFUL_RETURN) {
    exit(EXIT_FAILURE);
  }

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}