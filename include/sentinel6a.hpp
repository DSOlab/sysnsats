#ifndef __DSO_SENTINEL6A_MACROMODEL_HPP__
#define __DSO_SENTINEL6A_MACROMODEL_HPP__

/* @file
 *
 * Sentinel-6A (MICHAEL FREILICH) Macromodel according to:
 * L. Cerri, A. Couhert, P. Ferrage, DORIS satellites models implemented in
 * POE processing, available at:
 * https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf
 * Version: Ed.Rev.Date 1.19.21/03/2025
 *
 * We adopt here the model given in [1], i.e. Conrad’s 12-surface model
 * (except infrared properties). Note that the satellite actually does have
 * (two) solar panels, but we pretend it does not for the macromodel, i.e.
 * they are treated as fixed (i.e. not independently rotating) body frame
 * surfaces.
 *
 * See Conrad, A., Axelrad, P., Desai, S. et al. (2022). Improved modeling of
 * the solar radiation pressure for the Sentinel-6 MF spacecraft. In:
 * Proceedings of the 35th International Technical Meeting of the Satellite
 * Division of The Institute of Navigation (ION GNSS+ 2022) (pp. 3618–3631).
 * https://doi.org/10.33012/2022.18478
 *
 * See also: Daniel Calliess, Oliver Montenbruck, Martin Wermuth, Heinz
 * Reichinger, Long-term analysis of Sentinel-6A orbit determination: Insights
 * from three years of flight data, Advances in Space Research, Volume 74,
 * Issue 7, 2024, Pages 3011-3027, ISSN 0273-1177,
 * https://doi.org/10.1016/j.asr.2024.06.043.
 *
 * Measured attitude (quaternions) are published by COPERNICUS to describe
 * the (body frame) attitude.
 *
 * Hence, to get e.g. the normal vector of each surface j from the body frame
 * (B) to the inertial frame (I):
 *  * j belongs to the body frame: n_I = q * n_B
 */

#include "macromodel_surface_element.hpp"
#include "satellite_macromodel_traits.hpp"
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::SENTINEL6A> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 1;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 0;
}; /*SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>*/

template <> struct SatelliteMacromodelTraits<SATELLITE::SENTINEL6A> {

  static constexpr std::array<MacromodelSurfaceElement, 12> model = {{
      {4.149e0, 1.e0, 0.e0, 0.e0, 0.349e0, 0.041e0, 0.610e0, 0.100e0, 0.800e0,
       0.100},
      {3.941e0, -1.e0, 0.e0, 0.e0, 0.546e0, 0.042e0, 0.412e0, 0.100e0, 0.800e0,
       0.100},
      {11.83e0, 0.e0, 0.e0, 1.e0, 0.571e0, 0.016e0, 0.413e0, 0.100e0, 0.800e0,
       0.100},
      {2.072e0, 0.e0, 0.e0, -1.e0, 0.660e0, 0.030e0, 0.310e0, 0.100e0, 0.800e0,
       0.100},
      {8.65e0, 0.e0, 0.616e0, -0.788e0, 0.139e0, 0.316e0, 0.545e0, 0.100e0,
       0.800e0, 0.100},
      {8.65e0, 0.e0, -0.616e0, -0.788e0, 0.139e0, 0.316e0, 0.545e0, 0.100e0,
       0.800e0, 0.100},
      {3.76e0, 0.e0, 0.616e0, 0.788e0, 0.013e0, 0.164e0, 0.823e0, 0.100e0,
       0.800e0, 0.100},
      {3.76e0, 0.e0, -0.616e0, 0.788e0, 0.013e0, 0.164e0, 0.823e0, 0.100e0,
       0.800e0, 0.100},
      {1.329e0, 0.e0, 1.e0, 0.e0, 0.506e0, 0.040e0, 0.454e0, 0.100e0, 0.800e0,
       0.100},
      {1.329e0, 0.e0, -1.e0, 0.e0, 0.506e0, 0.040e0, 0.454e0, 0.100e0, 0.800e0,
       0.100},
      {0.92e0, 0.469e0, 0.e0, -0.833e0, 0.000e0, 0.080e0, 0.920e0, 0.100e0,
       0.800e0, 0.100},
      {0.8123e0, 0.e0, 0.e0, 1.e0, 0.190e0, 0.560e0, 0.250e0, 0.100e0, 0.800e0,
       0.100},
  }};

  /* number of body-frame plates/surfaces in macromodel */
  static constexpr int num_body_frame_surfaces() { return 12; }

  /* number of solar array plates/surfaces in macromodel, per array */
  static constexpr int num_solar_array_surfaces() { return 0; }

  /* number of solar arrays */
  static constexpr int num_solar_arrays() { return 0; }

  /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_s1_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.6251e0, 0.3993e0, 0.9972e0;
    return p;
  }

  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.6251e0, 0.3993e0, 0.8282e0;
    return p;
  }

  /* Initial value of mass in [kg] */
  static constexpr double initial_mass() { return 1191.831e0; }

  /* Center of gravity coordinates in satellite reference frame [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> initial_cog() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.5274e0, -0.0073e0, 0.0373e0;
    return p;
  }

  /** @brief Rotate the macromodel normal vectors (e.g. body frame to Inertial).
   *
   * The macromodel contains normal vectors (per surface) in the body-frame ref.
   * system. Here, we are rotating these normal vectors to another ref. frame,
   * given the rotation quaternion. For the case of Sentinel-6A, we need one
   * quaternion, to rotate all (normal vectors of the) body frame surfaces.
   *
   * The returned macromodel will contain:
   * - all body-frame surfaces, with their normal vectors rotated according to
   * the given quaternion; the number of surfaces is num_body_frame_surfaces()
   *
   * @param[in] qbody A pointer to Eigen::Quaterniond instances. Here, we only
   * need one. This is the quaternion used to rotate the body frame (for each
   * surface normal vector n_bf we apply n = q * n_bf).
   * @param[in] thetas Not used
   * @param[in] satsun Not used
   * @return A vector of MacromodelSurfaceElement's. Its size should equal:
   * num_body_frame_surfaces() + num_solar_array_surfaces() * num_solar_arrays()
   *
   */
  static std::vector<MacromodelSurfaceElement>
  rotate_macromodel(const Eigen::Quaterniond *qbody,
                    [[maybe_unused]] const double *,
                    [[maybe_unused]] const Eigen::Vector3d *) noexcept {

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

    return rotated;
  }
}; /* MacroModel<SATELLITE::SENTINEL6A> */

} /* namespace dso */

#endif
