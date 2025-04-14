#ifndef __DSO_JASON2_MACROMODEL_HPP__
#define __DSO_JASON2_MACROMODEL_HPP__

/* @file
 *
 * Jason-2 Macromodel according to:
 * L. Cerri, A. Couhert, P. Ferrage, DORIS satellites models implemented in
 * POE processing, available at:
 * https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf
 * Version: Ed.Rev.Date 1.19.21/03/2025
 *
 * Jason-2 has two solar panels, which can rotate w.r.t the Y-axis (Bframe).
 * The attitude is distributed by CNES via:
 *  * A body-frame quaternion, and
 *  * Left- and Right- panel rotation angles.
 *
 * Hence, to get e.g. the normal vector of each surface j from the body frame
 * (B) to the inertial frame (I):
 *  * j belongs to the body frame: n_I = q * n_B
 *  * j is either of the panels: n_I = q * ( R2(theta) * n_B )
 *    where theta is rotation angle for the left or right solar array.
 *
 * Note that the macromodel only has reference values for one panel; when we
 * rotate the macromodel though, we get two.
 */

#include "satellites/macromodel_core.hpp"
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::JASON2> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 1;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 2;
}; /*SatelliteAttitudeTraits<SATELLITE::JASON1>*/

template <> struct SatelliteMacromodelTraits<SATELLITE::JASON2> {

  static constexpr std::array<MacromodelSurfaceElement, 8> model = {
      {{0.783e0, -1e0, 0e0, 0e0, 0.3410e0, 0.6460e0, 0.0130e0, 0.0000e0,
        0.9870e0, 0.0130e0},
       {0.783e0, 1e0, 0e0, 0e0, 0.1490e0, 0.8510e0, 0.0000e0, 0.0000e0,
        1.0000e0, 0.0000e0},
       {2.040e0, 0e0, -1e0, 0e0, 0.5730e0, 0.3840e0, 0.0430e0, 0.1040e0,
        0.5690e0, 0.3280e0},
       {2.040e0, 0e0, 1e0, 0e0, 0.5390e0, 0.4240e0, 0.0370e0, 0.0890e0,
        0.6270e0, 0.2830e0},
       {3.105e0, 0e0, 0e0, -1e0, 0.2460e0, 0.7520e0, 0.0020e0, 0.0050e0,
        0.9770e0, 0.0170e0},
       {3.105e0, 0e0, 0e0, 1e0, 0.2130e0, 0.4530e0, 0.3340e0, 0.0370e0,
        0.2870e0, 0.6760e0},
       /* solar array */
       {9.800e0, 1e0, 0e0, 0e0, 0.0600e0, 0.4070e0, 0.5330e0, 0.0970e0,
        0.0980e0, 0.8030e0},
       {9.800e0, -1e0, 0e0, 0e0, 0.0040e0, 0.2980e0, 0.6970e0, 0.0350e0,
        0.0350e0, 0.9310e0}}};

  /* number of body-frame plates in macromodel */
  static constexpr int num_plates() { return 6; }

  /* number of solar array plates in macromodel */
  static constexpr int num_solar_arrays() { return 2; }

  /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_s1_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.194e0, -0.598e0, 1.022e0;
    return p;
  }

  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.194e0, -0.598e0, 0.858e0;
    return p;
  }

  /* Initial value of mass in [kg] */
  static constexpr double initial_mass() { return 505.9e0; }

  /* Center of gravity coordinates in satellite reference frame [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> initial_cog() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 0.9768e0, 0.0001e0, 0.0011e0;
    return p;
  }
}; /* MacroModel<SATELLITE::Jason2> */

} /* namespace dso */

#endif
