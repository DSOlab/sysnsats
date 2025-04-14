#ifndef __DSO_SATELLITE_MACROMODEL_SURFACE_ELEMENT_HPP__
#define __DSO_SATELLITE_MACROMODEL_SURFACE_ELEMENT_HPP__

#include "eigen3/Eigen/Eigen"

namespace dso {

struct MacromodelSurfaceElement {
  /* mpool = [0 1 2 3 4 5 6 7 8 9]
   *          0: area
   *            [1:3]: normal vector in satellite ref. frame [m]
   *                  [4:6]: optical properties in the order: spec, diff, abs
   *                        [7:9]: infrared properties in the order: spec,
   *                               diff, abs
   */
  double mpool[10];

  /* @brief Return the normal vector in the satellite body frame [m].
   *
   * Const version (returns a const Eigen::Vector3d reference to mpool[1:3])
   */
  Eigen::Map<const Eigen::Vector3d> normal() const {
    return Eigen::Map<const Eigen::Vector3d>(mpool + 1);
  }

  /* @brief Return the normal vector in the satellite body frame [m].
   *
   * Non-const version (allows writing into mpool[1:3])
   */
  Eigen::Map<Eigen::Vector3d> normal() {
    return Eigen::Map<Eigen::Vector3d>(mpool + 1);
  }
  double area() const noexcept { return mpool[0]; }
  double &area() noexcept { return mpool[0]; }
  double spec_optical() const noexcept { return mpool[0 + 4]; }
  double &spec_optical() noexcept { return mpool[0 + 4]; }
  double diff_optical() const noexcept { return mpool[1 + 4]; }
  double &diff_optical() noexcept { return mpool[1 + 4]; }
  double abs_optical() const noexcept { return mpool[2 + 4]; }
  double &abs_optical() noexcept { return mpool[2 + 4]; }
  double spec_infrared() const noexcept { return mpool[0 + 7]; }
  double &spec_infrared() noexcept { return mpool[0 + 7]; }
  double diff_infrared() const noexcept { return mpool[1 + 7]; }
  double &diff_infrared() noexcept { return mpool[1 + 7]; }
  double abs_infrared() const noexcept { return mpool[2 + 7]; }
  double &abs_infrared() noexcept { return mpool[2 + 7]; }
}; /* class MacromodelSurfaceElement */

} /* namespace dso */

#endif
