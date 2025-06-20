#ifndef __DSO_CRYOSAT2_MACROMODEL_HPP__
#define __DSO_CRYOSAT2_MACROMODEL_HPP__

/* @file
 *
 * Jason-3 Macromodel according to:
 * L. Cerri, A. Couhert, P. Ferrage, DORIS satellites models implemented in
 * POE processing, available at:
 * https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf
 * Version: Ed.Rev.Date 1.19.21/03/2025
 *
 * Jason-3 has two solar panels, which can rotate w.r.t the Y-axis (Bframe).
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

#include "macromodel_surface_element.hpp"
#include "satellite_macromodel_traits.hpp"
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::CRYOSAT2> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 1;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 2;
}; /*SatelliteAttitudeTraits<SATELLITE::CRYOSAT2>*/

template <> struct SatelliteMacromodelTraits<SATELLITE::CRYOSAT2> {
  static constexpr std::array<MacromodelSurfaceElement, 8> model = {{
      {0.783e0, -1.e0, 0.e0, 0.e0, 0.2000e0, 0.7000e0, 0.1000e0, 0.0000e0,
       0.9870e0, 0.0130},
      {0.783e0, 1.e0, 0.e0, 0.e0, 0.2000e0, 0.4000e0, 0.4000e0, 0.0000e0,
       1.0000e0, 0.0000},
      {2.040e0, 0.e0, -1.e0, 0.e0, 0.5730e0, 0.3840e0, 0.0430e0, 0.1040e0,
       0.5690e0, 0.3280},
      {2.040e0, 0.e0, 1.e0, 0.e0, 0.5390e0, 0.4240e0, 0.0370e0, 0.0890e0,
       0.6270e0, 0.2830},
      {3.105e0, 0.e0, 0.e0, -1.e0, 0.2460e0, 0.7520e0, 0.0020e0, 0.0050e0,
       0.9770e0, 0.0170},
      {3.105e0, 0.e0, 0.e0, 1.e0, 0.2130e0, 0.4530e0, 0.3340e0, 0.0370e0,
       0.2870e0, 0.6760},
      {9.8e0, 1.e0, 0.e0, 0.e0, 0.1000e0, 0.2950e0, 0.6050e0, 0.0970e0,
       0.0980e0, 0.8030},
      {9.8e0, -1.e0, 0.e0, 0.e0, 0.1000e0, 0.3000e0, 0.6000e0, 0.0350e0,
       0.0350e0, 0.9310},
  }};

  /* mean Area in [m^2] for computing SRP with cannonball model */
  static constexpr double srp_cannonball_area() {
    return 9.8 * 2. + (0.783 + 2.);
  };

  /* number of body-frame plates/surfaces in macromodel */
  static constexpr int num_body_frame_surfaces() { return 6; }

  /* number of solar array plates/surfaces in macromodel, per array */
  static constexpr int num_solar_array_surfaces() { return 2; }

  /* number of solar arrays */
  static constexpr int num_solar_arrays() { return 2; }

  /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_s1_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 2.4128e0, -0.1325e0, 0.9235e0;
    return p;
  }

  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 2.4128e0, -0.1325e0, 0.7555e0;
    return p;
  }

  /* Initial value of mass in [kg] */
  static constexpr double initial_mass() { return 509.6e0; }

  /* Center of gravity coordinates in satellite reference frame [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> initial_cog() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 1.0023e0, 0.0000e0, -0.0021e0;
    return p;
  }

  /** @brief Rotate the macromodel normal vectors (e.g. body frame to Inertial).
   *
   * The macromodel contains normal vectors (per surface) in the body-frame ref.
   * system. Here, we are rotating these normal vectors to another ref. frame,
   * given the rotation angles/quaternions.
   *
   * For the case of Jason-3 (as in all Jason satellite series), we need one
   * quaternion, to rotate all (normal vectors of the) body frame surfaces and
   * two angles (w.r.t the body-frame y-axis) to rotate the left and the right
   * solar arrays. Note that the macromodel only contains info for one solar
   * array, but the seconds is just a replica (though it has to be rotated by a
   * different angle, probably just the opsosite of the other).
   *
   * The returned macromodel will contain:
   * - all body-frame surfaces, with their normal vectors rotated according to
   * the given quaternion; the number of surfaces is num_body_frame_surfaces()
   * - the left solar-array, with its normal vectors rotated according to the
   * given quaternion and the (first) rotation angle, and
   * - the rigth solar-array, with its normal vectors rotated according to the
   * given quaternion and the (second) rotation angle.
   *
   * For each panel we have two surfaces (according to the reference
   * macromodel). Hence, the rotated macromodel will have a total of 4 surfaces,
   * 2 for each solar array. The left panel is placed before the right one, in
   * accordance to the order of the given rotation angles (in thetas).
   *
   * @param[in] qbody A pointer to Eigen::Quaterniond instances. Here, we only
   * need one. This is the quaternion used to rotate the body frame (for each
   * surface normal vector n_bf we apply n = q * n_bf).
   * @param[in] thetas Rotation angles, around the y-axis, for the solar arrays
   * [rad]. First, i.e. thetas[0] is the angle for the left panel, and thetas[1]
   * is the angle for the right panel. For e.g. the left panel, the rotation is:
   * n = q * (Ry(theta) * n_bf)
   * @param[in] satsun Not used
   * @return A vector of MacromodelSurfaceElement's. Its size should equal:
   * num_body_frame_surfaces() + num_solar_array_surfaces() * num_solar_arrays()
   *
   */
  static std::vector<MacromodelSurfaceElement> rotate_macromodel(
      const Eigen::Quaterniond *qbody, const double *thetas,
      [[maybe_unused]] const Eigen::Vector3d * = nullptr) noexcept {

    /* resulting rotated macromodel (add one solar array) */
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

    /* left solar array, 2 surfaces, rotate around y-axis with theta[0] */
    for (int i = 0; i < num_solar_array_surfaces(); i++) {
      rotated.emplace_back(*it);
      rotated[num_body_frame_surfaces() + i].normal() =
          (*qbody) * (Eigen::AngleAxisd(thetas[0], Eigen::Vector3d::UnitY()) *
                      it->normal());
      ++it;
    }

    /* right solar array, 2 surfaces, rotate around y-axis with theta[1] */
    it -= num_solar_array_surfaces();
    for (int i = 0; i < num_solar_array_surfaces(); i++) {
      rotated.emplace_back(*it);
      rotated[num_body_frame_surfaces() + num_solar_array_surfaces() + i]
          .normal() =
          (*qbody) * (Eigen::AngleAxisd(thetas[1], Eigen::Vector3d::UnitY()) *
                      it->normal());
      ++it;
    }
    return rotated;
  }
}; /* MacroModel<SATELLITE::Jason3> */

} /* namespace dso */

#endif
