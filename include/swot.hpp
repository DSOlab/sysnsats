#ifndef __DSO_SWOT_MACROMODEL_HPP__
#define __DSO_SWOT_MACROMODEL_HPP__

/* @file
 *
 * Swot Macromodel according to:
 * L. Cerri, A. Couhert, P. Ferrage, DORIS satellites models implemented in
 * POE processing, available at:
 * https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf
 * Version: Ed.Rev.Date 1.19.21/03/2025
 *
 */

#include "macromodel_surface_element.hpp"
#include "satellite_macromodel_traits.hpp"
#include <vector>

namespace dso {

template <> struct SatelliteAttitudeTraits<SATELLITE::SWOT> {
  /** Number of quaternions in measured attitude files. */
  static constexpr int NumQuaternions = 1;
  /** Number of angles in measured attitude files. */
  static constexpr int NumAngles = 2;
}; /*SatelliteAttitudeTraits<SATELLITE::SWOT>*/

template <> struct SatelliteMacromodelTraits<SATELLITE::SWOT> {
  static constexpr std::array<MacromodelSurfaceElement, 8> model = {{
{7.880e+00,1.000e+00,0.000e+00,0.000e+00,3.600e-01,6.300e-01,1.000e-02,2.000e-02,9.200e-01,6.000e-02},
{7.880e+00,-1.000e+00,0.000e+00,0.000e+00,3.900e-01,6.000e-01,1.000e-02,4.000e-02,8.600e-01,1.000e-01},
{1.219e+01,0.000e+00,1.000e+00,0.000e+00,5.600e-01,3.600e-01,8.000e-02,1.400e-01,5.200e-01,3.300e-01},
{1.219e+01,0.000e+00,-1.000e+00,0.000e+00,3.100e-01,6.900e-01,0.000e+00,0.000e+00,1.000e+00,0.000e+00},
{7.330e+00,0.000e+00,0.000e+00,1.000e+00,3.100e-01,6.900e-01,0.000e+00,0.000e+00,1.000e+00,0.000e+00},
{7.330e+00,0.000e+00,0.000e+00,-1.000e+00,3.100e-01,6.900e-01,0.000e+00,0.000e+00,1.000e+00,0.000e+00},
/* Solar arrays in the frame Xsa1,Ysa1,Zsa1 */
{3.138e+01,0.000e+00,0.000e+00,1.000e+00,0.000e+00,1.000e-01,9.000e-01,0.000e+00,3.000e-01,7.000e-01},
{3.138e+01,0.000e+00,0.000e+00,-1.000e+00,1.300e-01,0.000e+00,8.700e-01,0.000e+00,2.000e-01,8.000e-01}
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
    p << 0.0002e0, 0.5441e0, 2.7777e0;
    return p;
  }

  /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 0.0002e0, 0.5441e0, 2.6117e0;
    return p;
  }

  /* Initial value of mass in [kg] */
  static constexpr double initial_mass() { return 2103.506; }

  /* Center of gravity coordinates in satellite reference frame [m] (xyz) */
  static Eigen::Matrix<double, 3, 1> initial_cog() noexcept {
    Eigen::Matrix<double, 3, 1> p;
    p << 0.0015e0, 0.0086e0, -0.5921e0;
    return p;
  }

  /** @brief Rotate the macromodel normal vectors (e.g. body frame to Inertial).
   *
   * The macromodel contains normal vectors (per surface) in the body-frame ref.
   * system. Here, we are rotating these normal vectors to another ref. frame,
   * given the rotation angles/quaternions.
   *
   * For the case of Swot (as in Jason satellite series), we need one
   * quaternion, to rotate all (normal vectors of the) body frame surfaces and
   * two angles (w.r.t the body-frame x-axis) to rotate the left and the right
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
   * @param[in] thetas Rotation angles, around the x-axis, for the solar arrays
   * [rad]. First, i.e. thetas[0] is the angle for the 1/left panel, and 
   * thetas[1] is the angle for the 2/right panel. For e.g. the left panel, 
   * the rotation is: n = q * (Ry(theta) * n_bf)
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
}; /* MacroModel<SATELLITE::SWOT> */

} /* namespace dso */

#endif
