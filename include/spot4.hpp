#ifndef __DSO_SPOT4_MACROMODEL_HPP__
#define __DSO_SPOT4_MACROMODEL_HPP__

/* @file
 *
 * Spot-2 Macromodel according to:
 * L. Cerri, A. Couhert, P. Ferrage, DORIS satellites models implemented in
 * POE processing, available at:
 * https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf
 * Version: Ed.Rev.Date 1.19.21/03/2025
 */

#include "macromodel_surface_element.hpp"
#include "satellite_macromodel_traits.hpp"
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::SPOT4> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 0;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 0;
}; /*SatelliteAttitudeTraits<SATELLITE::SPOT4>*/

template <> struct SatelliteMacromodelTraits<SATELLITE::SPOT4> {
  static constexpr std::array<MacromodelSurfaceElement, 8> model = {
      {{35000E+00, 1.0000E+00, 0.0000E+00, 0.0000E+00, 1.0000E+00, -3.8000E-01,
        3.8000E-01, 2.1000E-01, 3.0000E-02, 7.6000E-01},
       {3.5000E+00, -1.0000E+00, 0.0000E+00, 0.0000E+00, 6.3000E-01, 8.1000E-01,
        -4.4000E-01, 2.1000E-01, 3.0000E-02, 7.6000E-01},
       {7.7000E+00, 0.0000E+00, 1.0000E+00, 0.0000E+00, 5.6000E-01, 3.8000E-01,
        6.0000E-02, 2.2000E-01, 3.0000E-02, 7.5000E-01},
       {7.7000E+00, 0.0000E+00, -1.0000E+00, 0.0000E+00, 5.4000E-01, 5.0000E-01,
        -4.0000E-02, 2.2000E-01, 3.0000E-02, 7.5000E-01},
       {9.0000E+00, 0.0000E+00, 0.0000E+00, 1.0000E+00, 4.7000E-01, 1.1000E-01,
        5.2000E-01, 2.6000E-01, 4.0000E-02, 7.0000E-01},
       {9.0000E+00, 0.0000E+00, 0.0000E+00, -1.0000E+00, 4.7000E-01, 2.5000E-01,
        2.8000E-01, 2.6000E-01, 4.0000E-02, 7.0000E-01},
       /* solar array normals are: a. to sun, b) opposite to sun */
       {2.4800E+01, 0.0000E+00, 0.0000E+00, 0.0000E+00, 1.0000E-01, 1.5000E-01,
        7.5000E-01, 1.0000E-01, 6.0000E-02, 8.4000E-01},
       {2.4800E+01, 0.0000E+00, 0.0000E+00, 0.0000E+00, 2.4000E-01, 2.4000E-01,
        5.2000E-01, 1.0000E-01, 6.0000E-02, 8.4000E-01}}};

  /* mean Area in [m^2] for computing SRP with cannonball model */
  static constexpr double srp_cannonball_area() {
    return 9.8 * 2. + (0.783 + 2.);
  }

  /* number of body-frame plates/surfaces in macromodel */
  static constexpr int num_body_frame_surfaces() { return 6; }

  /* number of solar array plates/surfaces in macromodel, per array */
  static constexpr int num_solar_array_surfaces() { return 2; }

  /* number of solar arrays */
  static constexpr int num_solar_arrays() { return 1; }

  /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_s1_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << -0.770e0, -0.330e0, -1.266e0;
    return p;
  }

  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz)
   */
  static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << -0.770e0, -0.330e0, -1.105e0;
    return p;
  }

  /* Initial value of mass in [kg] */
  static constexpr double initial_mass() { return 2753.960; }

  /* Center of gravity coordinates in satellite reference frame [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> initial_cog() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << -1.901e0, 0.008e0, 0.059e0;
    return p;
  }

  /** @brief Rotate the macromodel normal vectors (e.g. body frame to
   * Inertial).
   *
   * The macromodel contains normal vectors (per surface) in the body-frame
   * ref. system. Here, we are rotating these normal vectors to another ref.
   * frame, given the rotation angles/quaternions.
   *
   * For the case of Jason-3 (as in all Jason satellite series), we need one
   * quaternion, to rotate all (normal vectors of the) body frame surfaces and
   * two angles (w.r.t the body-frame y-axis) to rotate the left and the right
   * solar arrays. Note that the macromodel only contains info for one solar
   * array, but the seconds is just a replica (though it has to be rotated by
   * a different angle, probably just the opsosite of the other).
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
   * macromodel). Hence, the rotated macromodel will have a total of 4
   * surfaces, 2 for each solar array. The left panel is placed before the
   * right one, in accordance to the order of the given rotation angles (in
   * thetas).
   *
   * @param[in] qbody Not used
   * @param[in] thetas  Not used
   * @param[in] vecs A pointer to three vectors:
   * vecs[0] satellite-to-sun vector (does not have to be normalized)
   * vecs[1] Satellite position in geocentric inertial frame
   * vecs[2] Satellite velocity in geocentric inertial frame
   * @return A vector of MacromodelSurfaceElement's. Its size should equal:
   * num_body_frame_surfaces() + num_solar_array_surfaces() *
   * num_solar_arrays()
   *
   */
  static std::vector<MacromodelSurfaceElement>
  rotate_macromodel([[maybe_unused]] const Eigen::Quaterniond *,
                    [[maybe_unused]] const double *,
                    const Eigen::Vector3d *vecs) noexcept {

    /* The satellite Z axis is oriented as the radial direction. */
    const Eigen::Vector3d Zsat = vecs[1].normalized();
    /* The satellite X axis is oriented as the cross-track direction */
    const Eigen::Vector3d Xsat = vecs[1].cross(vecs[2]).normalized();
    /* The satellite Y axis is oriented opposite to the along-track direction
     */
    const Eigen::Vector3d Ysat = -Xsat.cross(Zsat);

    /* transformation matrix from R_inertial = M * R_bf */
    Eigen::Matrix3d M;
    M.col(0) = Xsat;
    M.col(1) = Ysat;
    M.col(2) = Zsat;

    /* resulting rotated macromodel (add solar array) */
    std::vector<MacromodelSurfaceElement> rotated;
    rotated.reserve(num_body_frame_surfaces() +
                    num_solar_array_surfaces() * num_solar_arrays());

    /* iterator to model (every plate) */
    auto it = model.cbegin();

    /* body frame */
    for (int i = 0; i < num_body_frame_surfaces(); i++) {
      rotated.emplace_back(*it);
      /* apply rotation via rotation matrix */
      rotated[i].normal() = M * it->normal();
      ++it;
    }

    /* solar array, 2 surfaces */
    for (int i = 0; i < num_solar_array_surfaces(); i++) {
      rotated.emplace_back(*it);
      int sign = 1 | -(i & 1); // 1 for even, -1 for odd
      rotated[num_body_frame_surfaces() + i].normal() =
          sign * vecs[0].normalized();
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
  static Eigen::Quaterniond
  bodyframe2inertial([[maybe_unused]] const Eigen::Quaterniond *,
                     [[maybe_unused]] const double *,
                     const Eigen::Vector3d *vecs) noexcept {
    /* The satellite Z axis is oriented as the radial direction. */
    const Eigen::Vector3d Zsat = vecs[1].normalized();
    /* The satellite X axis is oriented as the cross-track direction */
    const Eigen::Vector3d Xsat = vecs[1].cross(vecs[2]).normalized();
    /* The satellite Y axis is oriented opposite to the along-track direction
     */
    const Eigen::Vector3d Ysat = -Xsat.cross(Zsat);

    /* transformation matrix from R_inertial = M * R_bf */
    Eigen::Matrix3d M;
    M.col(0) = Xsat;
    M.col(1) = Ysat;
    M.col(2) = Zsat;
    /* convert rotation matrix -> quaternion */
    Eigen::Quaterniond q(M);
    /* good practice if R may have small numerical errors */
    q.normalize();
    return q;
  }
}; /* MacroModel<SATELLITE::SPOT4> */

} /* namespace dso */

#endif
