#ifndef __DSO_SENTINEL3B_MACROMODEL_HPP__
#define __DSO_SENTINEL3B_MACROMODEL_HPP__

#include "satellites/macromodel.hpp"
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::SENTINEL3B> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 1;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 0;
}; /*SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>*/

template <> struct SatelliteMacromodelTraits<SATELLITE::SENTINEL3B> {

  static constexpr std::array<MacromodelSurfaceElement, 8> model = {
      {{1.95e0, 1e0, 0e0, 0e0, 0.079e0, 0.906e0, 0.015e0, 0.079e0, 0.847e0,
        0.015e0},
       {1.95e0, -1e0, 0e0, 0e0, 0.089e0, 0.908e0, 0.003e0, 0.090e0, 0.850e0,
        0.001e0},
       {4.68e0, 0e0, 1e0, 0e0, 0.290e0, 0.685e0, 0.026e0, 0.126e0, 0.640e0,
        0.189e0},
       {4.68e0, 0e0, -1e0, 0e0, 0.400e0, 0.558e0, 0.042e0, 0.149e0, 0.522e0,
        0.292e0},
       {5.40e0, 0e0, 0e0, 1e0, 0.106e0, 0.712e0, 0.183e0, 0.084e0, 0.603e0,
        0.274e0},
       {5.40e0, 0e0, 0e0, -1e0, 0.351e0, 0.615e0, 0.034e0, 0.139e0, 0.575e0,
        0.246e0},
       /* solar arrays */
       {10.5e0, 1e0, 0e0, 0e0, 0.180e0, 0.082e0, 0.738e0, 0.310e0, 0.069e0,
        0.621e0},
       {10.5e0, -1e0, 0e0, 0e0, 0.000e0, 0.109e0, 0.729e0, 0.000e0, 0.197e0,
        0.657e0}}};

  /* number of body-frame plates in macromodel */
  static constexpr int num_plates() { return 6; }

  /* number of solar array plates in macromodel */
  static constexpr int num_solar_arrays() { return 2; }

  /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_s1_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.570e0, 0.073e0, 1.076e0;
    return p;
  }

  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.570e0, 0.073e0, 0.910e0;
    return p;
  }

  /* Initial value of mass in [kg] */
  static constexpr double initial_mass() { return 1130e0; }

  /* Center of gravity coordinates in satellite reference frame [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> initial_cog() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.4888e0, 0.2174e0, 0.0094e0;
    return p;
  }
}; /* MacroModel<SATELLITE::SENTINEL3B> */

} /* namespace dso */

#endif
