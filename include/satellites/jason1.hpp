#ifndef __DSO_JASON1_MACROMODEL_HPP__
#define __DSO_JASON1_MACROMODEL_HPP__

/* @file
 * Jaon-1 Macromodel according to:
 * L. Cerri, A. Couhert, P. Ferrage, DORIS satellites models implemented in
 * POE processing, available at:
 * https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf
 * Version: Ed.Rev.Date 1.19.21/03/2025
 *
 * Jason-1 has two solar panels, which can rotate w.r.t the Y-axis (Bframe).
 * The attitude is distributed by CNES via:
 *  * A body-frame quaternion, and
 *  * Left- and Right- panel rotation angles.
 *
 * Hence, to get e.g. the norma vector of each surface j from the body frame
 * (B) to the inertial frame (I):
 *  * j belongs to the body frame: n_I = q * n_B
 *  * j is either of the panels: n_I = q * ( R2(theta) * n_B )
 *    where theta is rotation angle for the left or right solar array.
 *
 */

#include "core/macromodel_surface_element.hpp"
#include "satellites/satellites_core.hpp"
#include <array>
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::JASON1> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 1;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 2;
}; /*SatelliteAttitudeTraits<SATELLITE::JASON1>*/

template <>
struct SatelliteMacromodel<SATELLITE::JASON1>
    : public satellite_details::BaseMacromodel {
  static constexpr std::array<MacromodelSurfaceElement, 8> model = {
      {{1.65e0, 1e0, 0e0, 0e0, 0.0938e0, 0.2811e0, 0.2078e0, 0.4250e0, 0.1780e0,
        -.0260},
       {1.65e0, -1e0, 0e0, 0e0, 0.4340e0, 0.2150e0, 0.0050e0, 0.4080e0,
        0.1860e0, -.0120},
       {3.e0, 0e0, 1e0, 0e0, 1.1880e0, -.0113e0, -.0113e0, 0.3340e0, 0.3420e0,
        0.2490},
       {3.e0, 0e0, -1e0, 0e0, 1.2002e0, -.0044e0, -.0044e0, 0.2740e0, 0.3690e0,
        0.2970},
       {3.1e0, 0e0, 0e0, 1e0, 0.2400e0, 0.4020e0, 0.3300e0, 0.2360e0, 0.3820e0,
        0.3090},
       {3.1e0, 0e0, 0e0, -1e0, 0.3180e0, 0.3700e0, 0.2670e0, 0.2980e0, 0.3360e0,
        0.2400},
       /* solar array */
       {9.8e0, 1e0, 0e0, 0e0, 0.1940e0, 0.0060e0, 0.9470e0, 0.0970e0, 0.0980e0,
        0.8030e0},
       {9.8e0, -1e0, 0e0, 0e0, 0.0040e0, 0.2980e0, 0.6970e0, 0.0350e0, 0.0350e0,
        0.9310e0}}};

  std::vector<MacromodelSurfaceElement>
  rotate_macromodel(Eigen::Quaterniond &qbody,
                    const double *thetas) const noexcept {
    std::vector<MacromodelSurfaceElement> rotated;
    rotated.reserve(model.size());
    /* iterator to model (every plate) */
    auto it = model.cbegin();

    /* body frame */
    for (int i = 0; i < num_plates(); i++) {
      rotated.emplace_back(*it);
      rotated[i].normal() = qbody * it->normal();
      ++it;
    }

    /* solar array */
    for (int i = 0; i < num_solar_arrays(); i++) {
      rotated.emplace_back(*it);
      rotated[num_plates() + i].normal() =
          qbody * (Eigen::AngleAxisd(thetas[i], Eigen::Vector3d::UnitY()) *
                   it->normal());
      ++it;
    }

    return rotated;
  }

  /* number of body-frame plates in macromodel */
  static constexpr int num_plates() { return 6; }

  /* number of solar array plates in macromodel */
  static constexpr int num_solar_arrays() { return 2; }

  /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_s1_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.171e0, -0.598e0, 1.027e0;
    return p;
  }

  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz)
   */
  static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.171, -0.598, 0.859e0;
    return p;
  }

  /* Initial value of mass in [kg] (note value extracted from ja1mass.txt,
   * available at ftp://ftp.ids-doris.org/pub/ids/satellites/) */
  static constexpr double initial_mass() { return 489.1e0; }

  /* Center of gravity coordinates in satellite reference frame [m] (xyz).
   * Note: extracted from ja1mass.txt, available at
   * ftp://ftp.ids-doris.org/pub/ids/satellites/
   */
  static Eigen::Matrix<double, 3, 1> initial_cog() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 0.955e0, 0e0, 0e0;
    return p;
  }
}; /* MacroModel<SATELLITE::Jason1> */

} /* namespace dso */

#endif
