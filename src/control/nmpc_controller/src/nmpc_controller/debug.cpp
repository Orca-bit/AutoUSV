#include <ostream>

#include "nmpc_controller/nmpc_controller.hpp"

namespace motion
{
namespace control
{
namespace nmpc_controller
{
using AcadosReal = real_t;

////////////////////////////////////////////////////////////////////////////////
void NmpcController::debug_print(std::ostream & out) const
{
  {
    out << "x0:\n";
    std::array<AcadosReal, NX> x0{};
    ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, 0, "x", x0.data());
    for (std::size_t jdx = {}; jdx < NX; ++jdx) {
      out << x0[jdx] << "\t";
    }
    out << "\n";
  }
  {
    out << "x:\n";
    std::array<AcadosReal, NX> x{};
    for (std::size_t idx = {}; idx < N; ++idx) {
      ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, idx + 1, "x", x.data());
      for (std::size_t jdx = {}; jdx < NX; ++jdx) {
        out << x[jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "u:\n";
    std::array<AcadosReal, NU> u{};
    for (std::size_t idx = {}; idx < N; ++idx) {
      ocp_nlp_out_get(m_nlp_config, m_nlp_dims, m_nlp_out, idx, "u", u.data());
      for (std::size_t jdx = {}; jdx < NU; ++jdx) {
        out << u[jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "y:\n";
    for (std::size_t idx = {}; idx < N; ++idx) {
      for (std::size_t jdx = {}; jdx < NY; ++jdx) {
        out << m_last_reference[(idx * NY) + jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "yN:\n";
    for (std::size_t jdx = {}; jdx < NYN; ++jdx) {
      out << m_last_terminal_reference[jdx] << "\t";
    }
    out << "\n";
  }
  // {
  //   out << "W:\n";
  //   for (std::size_t idx = {}; idx < N; ++idx) {
  //     for (std::size_t jdx = {}; jdx < NY; ++jdx) {
  //       out << W[(NY * NY * idx) + ((jdx * NY) + jdx)] << "\t";
  //     }
  //     out << "\n";
  //   }
  // }
  // {
  //   out << "WN:\n";
  //   for (std::size_t jdx = {}; jdx < NYN; ++jdx) {
  //     out << WN[(jdx * NYN) + jdx] << "\t";
  //   }
  //   out << "\n";
  // }
}
}  // namespace nmpc_controller
}  // namespace control
}  // namespace motion
