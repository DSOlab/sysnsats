#ifndef __DSO_JASON1_MACROMODEL_HPP__
#define __DSO_JASON1_MACROMODEL_HPP__

/* @file
 *
 * Jason-1 Macromodel according to:
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
 * Hence, to get e.g. the normal vector of each surface j from the body frame
 * (B) to the inertial frame (I):
 *  * j belongs to the body frame: n_I = q * n_B
 *  * j is either of the panels: n_I = q * ( R2(theta) * n_B )
 *    where theta is rotation angle for the left or right solar array.
 *
 * Note that the macromodel only has reference values for one panel; when we
 * rotate the macromodel though, we get two.
 *
 * For this macromodel, the optical coefficients do not add up to unity! I do
 * not know why that is.
 */

#include "macromodel_surface_element.hpp"
#include "satellite_macromodel_traits.hpp"
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::JASON1> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 1;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 2;
}; /* SatelliteAttitudeTraits<SATELLITE::JASON1> */

template <> struct SatelliteMacromodelTraits<SATELLITE::JASON1> {
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

  /* mean Area in [m^2] for computing SRP with cannonball model */
  static constexpr double srp_cannonball_area() {
    return 9.8 * 2. + (1.65 + 3.);
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

  /** @brief Rotate the macromodel normal vectors (e.g. body frame to Inertial).
   *
   * The macromodel contains normal vectors (per surface) in the body-frame ref.
   * system. Here, we are rotating these normal vectors to another ref. frame,
   * given the rotation angles/quaternions.
   *
   * For the case of Jason-1 (as in all Jason satellite series), we need one
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

  /* Return a (rotation) quaternion q, that transforms a body-fixed vector to
   * an inertial one, assuming we are on the satellite's body frame.

   * @param[in] qbody A pointer to Eigen::Quaterniond instances. Here, we only
   * need one. This is the quaternion used to rotate the body frame (for each
   * surface normal vector n_bf we apply n = q * n_bf).
   * @param[in] thetas  Not used
   * @param[in] satsun Not used
   *
   * @return A rotation quaternion that works in the sense:
   * r_inertial = q * r_bodyframe
   * assuming r_bodyframe is on the satellite's body frame
   */
  static Eigen::Quaterniond bodyframe2inertial(
      const Eigen::Quaterniond *qbody,
      [[maybe_unused]] const double *thetas = nullptr,
      [[maybe_unused]] const Eigen::Vector3d * = nullptr) noexcept {
    return *qbody;
  }
}; /* MacroModel<SATELLITE::Jason1> */

} /* namespace dso */

#endif
