#ifndef __DSO_SENTINEL3B_MACROMODEL_HPP__
#define __DSO_SENTINEL3B_MACROMODEL_HPP__

/* @file
 *
 * Sentinel-3B Macromodel according to:
 * L. Cerri, A. Couhert, P. Ferrage, DORIS satellites models implemented in
 * POE processing, available at:
 * https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf
 * Version: Ed.Rev.Date 1.19.21/03/2025
 *
 * Sentinel-3B has one solar panel (properties described in the macromodel).
 * According to [1], "The solar panel rotates around the Y axis in
 * order to be oriented as well as possible towards the sun. The plane of the
 * solar array is tilted of 24° from the rotation axis". This means that at
 * least as an approximation, the panel normal vector (in the inertial RF) can
 * be considered to be pointing in the direction satellite-to-sun.
 *
 * Measured attitude (quaternions) are published by COPERNICUS to describe
 * the (body frame) attitude.
 *
 * Hence, to get e.g. the normal vector of each surface j from the body frame
 * (B) to the inertial frame (I):
 *  * j belongs to the body frame: n_I = q * n_B
 *  * j belongs to the panel: n_I = (satellite-to-sun)/|(satellite-to-sun)|
 */

#include "macromodel_surface_element.hpp"
#include "satellite_macromodel_traits.hpp"
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
      {{1.95e0, 1.e0, 0.e0, 0.e0, 0.079e0, 0.906e0, 0.015e0, 0.079e0, 0.847e0,
        0.015},
       {1.95e0, -1.e0, 0.e0, 0.e0, 0.089e0, 0.908e0, 0.003e0, 0.090e0, 0.850e0,
        0.001},
       {4.68e0, 0.e0, 1.e0, 0.e0, 0.290e0, 0.685e0, 0.026e0, 0.126e0, 0.640e0,
        0.189},
       {4.68e0, 0.e0, -1.e0, 0.e0, 0.400e0, 0.558e0, 0.042e0, 0.149e0, 0.522e0,
        0.292},
       {5.40e0, 0.e0, 0.e0, 1.e0, 0.106e0, 0.712e0, 0.183e0, 0.084e0, 0.603e0,
        0.274},
       {5.40e0, 0.e0, 0.e0, -1.e0, 0.351e0, 0.615e0, 0.034e0, 0.139e0, 0.575e0,
        0.246},
       /* solar arrays */
       {10.5e0, 1.e0, 0.e0, 0.e0, 0.148e0, 0.114e0, 0.738e0, 0.310e0, 0.069e0,
        0.621},
       {10.5e0, -1.e0, 0.e0, 0.e0, 0.000e0, 0.109e0, 0.729e0, 0.000e0, 0.197e0,
        0.657}}};

  /* number of body-frame plates/surfaces in macromodel */
  static constexpr int num_body_frame_surfaces() { return 6; }

  /* number of solar array plates/surfaces in macromodel, per array */
  static constexpr int num_solar_array_surfaces() { return 2; }

  /* number of solar arrays */
  static constexpr int num_solar_arrays() { return 1; }

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

  /** @brief Rotate the macromodel normal vectors (e.g. body frame to Inertial).
   *
   * The macromodel contains normal vectors (per surface) in the body-frame ref.
   * system. Here, we are rotating these normal vectors to another ref. frame,
   * given the rotation angles/quaternions and the satellite-to-sun vector.
   *
   * For the case of Sentinel-3B, we need one quaternion, to rotate all
   * (normal vectors of the) body frame surfaces and the satellite-to-sun (in
   * the final, inertial RF) vector to rotate the panel. For the latter, we
   * consider that the normal vector of the panel is aligned to the
   * satellite-to-sun direction (in the final, inertial RF).
   *
   * The returned macromodel will contain:
   * - all body-frame surfaces, with their normal vectors rotated according to
   * the given quaternion; the number of surfaces is num_body_frame_surfaces()
   * - the solar-array, with its normal vectors aligned to the
   * satellite-to-sun direction (for the opposite surface this will be at the
   * opposite direction).
   *
   * @param[in] qbody A pointer to Eigen::Quaterniond instances. Here, we only
   * need one. This is the quaternion used to rotate the body frame (for each
   * surface normal vector n_bf we apply n = q * n_bf).
   * @param[in] thetas Not used
   * @param[in] satsun A 3-d cartesian vector in the satellite-to-sun
   * direction in the resulting (i.e. inertial) reference frame [m]. It does
   * not need to be normalized. This vector (normalized) will be used to
   * set the normal vector of the solar array.
   * @return A vector of MacromodelSurfaceElement's. Its size should equal:
   * num_body_frame_surfaces() + num_solar_array_surfaces() * num_solar_arrays()
   *
   */
  static std::vector<MacromodelSurfaceElement>
  rotate_macromodel(const Eigen::Quaterniond *qbody,
                    [[maybe_unused]] const double *,
                    const Eigen::Vector3d *satsun) noexcept {

    /* resulting rotated macromodel */
    std::vector<MacromodelSurfaceElement> rotated;
    rotated.reserve(num_body_frame_surfaces() +
                    num_solar_array_surfaces() * num_solar_arrays());

    /* iterator to model (every plate) */
    auto it = model.cbegin();

    /* body frame */
    for (int i = 0; i < num_body_frame_surfaces(); i++) {
      rotated.emplace_back(*it);
      /* apply rotation via quaternion */
      rotated[i].normal() = (*qbody) * it->normal();
      ++it;
    }

    /* solar array, 2 surfaces, normal vec is satellite-to-sun */
    for (int i = 0; i < num_solar_array_surfaces(); i++) {
      rotated.emplace_back(*it);
      rotated[num_body_frame_surfaces() + i].normal() = satsun->normalized();
      ++it;
    }

    return rotated;
  }
}; /* MacroModel<SATELLITE::SENTINEL3B> */

} /* namespace dso */

#endif
