// Treat as a system header since we don't want to touch that autogenerated stuff..
#include <acado_common.h>

#include <ostream>

#include "mpc_controller/mpc_controller.hpp"

namespace motion
{
namespace control
{
namespace mpc_controller
{

////////////////////////////////////////////////////////////////////////////////
void MpcController::debug_print(std::ostream & out) const
{
  {
    out << "x0:\n";
    for (std::size_t jdx = {}; jdx < ACADO_NX; ++jdx) {
      out << acadoVariables.x0[jdx] << "\t";
    }
    out << "\n";
  }
  {
    out << "x:\n";
    for (std::size_t idx = {}; idx < ACADO_N; ++idx) {
      for (std::size_t jdx = {}; jdx < ACADO_NX; ++jdx) {
        out << acadoVariables.x[(idx * ACADO_NX) + jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "u:\n";
    for (std::size_t idx = {}; idx < ACADO_N; ++idx) {
      for (std::size_t jdx = {}; jdx < ACADO_NU; ++jdx) {
        out << acadoVariables.u[(idx * ACADO_NU) + jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "y:\n";
    for (std::size_t idx = {}; idx < ACADO_N; ++idx) {
      for (std::size_t jdx = {}; jdx < ACADO_NY; ++jdx) {
        out << acadoVariables.y[(idx * ACADO_NY) + jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "yN:\n";
    for (std::size_t jdx = {}; jdx < ACADO_NYN; ++jdx) {
      out << acadoVariables.yN[jdx] << "\t";
    }
    out << "\n";
  }
  {
    out << "W:\n";
    for (std::size_t idx = {}; idx < ACADO_N; ++idx) {
      for (std::size_t jdx = {}; jdx < ACADO_NY; ++jdx) {
        out << acadoVariables.W[(ACADO_NY * ACADO_NY * idx) + ((jdx * ACADO_NY) + jdx)] <<
          "\t";
      }
      out << "\n";
    }
  }
  {
    out << "WN:\n";
    for (std::size_t jdx = {}; jdx < ACADO_NYN; ++jdx) {
      out << acadoVariables.WN[(jdx * ACADO_NYN) + jdx] << "\t";
    }
    out << "\n";
  }
}
}  // namespace mpc_controller
}  // namespace control
}  // namespace motion
