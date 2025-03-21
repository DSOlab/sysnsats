#ifndef __DSO_JASON3_MACROMODEL_HPP__
#define __DSO_JASON3_MACROMODEL_HPP__

#include "satellites.hpp"
#include "core/macromodel_surface_element.hpp"
#include <array>

namespace dso {
template <> struct SatelliteMacromodel<SATELLITE::JASON3> {
  static constexpr std::array<MacromodelSurfaceElement, 8> model = {{
      {0.783e0,
       {-1e0, 0e0, 0e0},
       {0.3410e0, 0.6460e0, 0.0130e0},
       {0.0000e0, 0.9870e0, 0.0130e0}},
      {0.783e0,
       {1e0, 0e0, 0e0},
       {0.1490e0, 0.8510e0, 0.0000e0},
       {0.0000e0, 1.0000e0, 0.0000e0}},
      {2.040e0,
       {0e0, -1e0, 0e0},
       {0.5730e0, 0.3840e0, 0.0430e0},
       {0.1040e0, 0.5690e0, 0.3280e0}},
      {2.040e0,
       {0e0, 1e0, 0e0},
       {0.5390e0, 0.4240e0, 0.0370e0},
       {0.0890e0, 0.6270e0, 0.2830e0}},
      {3.105e0,
       {0e0, 0e0, -1e0},
       {0.2460e0, 0.7520e0, 0.0020e0},
       {0.0050e0, 0.9770e0, 0.0170e0}},
      {3.105e0,
       {0e0, 0e0, 1e0},
       {0.2130e0, 0.4530e0, 0.3340e0},
       {0.0370e0, 0.2870e0, 0.6760e0}},
      /* solar array */
      {9.800e0,
       {1e0, 0e0, 0e0},
       {0.0600e0, 0.4070e0, 0.5330e0},
       {0.0970e0, 0.0980e0, 0.8030e0}},
      {9.800e0,
       {-1e0, 0e0, 0e0},
       {0.0040e0, 0.2980e0, 0.6970e0},
       {0.0350e0, 0.0350e0, 0.9310e0}}
  }};
  /* number of body-frame plates in macromodel */
  static constexpr int num_plates() {return 6;}
  /* number of solar array plates in macromodel */
  static constexpr int num_solar_arrays() {return 2;}
  /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double,3,1> doris_s1_pco() noexcept {
    Eigen::Matrix<double,3,1> p;
    p << 2.4128e0, -0.1325e0, 0.9235e0;
    return p;
  }
  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double,3,1> doris_u2_pco() noexcept {
    Eigen::Matrix<double,3,1> p;
    p << 2.4128e0, -0.1325e0, 0.7555e0;
    return p;
  }
  /* Initial value of mass in [kg] */
  static constexpr double initial_mass() {return 509.6e0;}
  /* Center of gravity coordinates in satellite reference frame [m] (xyz) */
  static Eigen::Matrix<double,3,1> initial_cog() noexcept {
    Eigen::Matrix<double,3,1> p;
    p << 1.0023e0, 0.0000e0, -0.0021e0;
    return p;
  }
}; /* MacroModel<SATELLITE::Jason3> */

} /* namespace dso */

#endif
