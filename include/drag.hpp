#ifndef __DSO_SATELLITE_ATMOSPHERIC_DRAG_ACCELERATION_HPP__
#define __DSO_SATELLITE_ATMOSPHERIC_DRAG_ACCELERATION_HPP__

#include "attitude.hpp"
#include "eigen3/Eigen/Eigen"

namespace dso {

Eigen::Vector3d
atmospheric_drag(const std::vector<MacromodelSurfaceElement> &macromodel_icf,
                 const Eigen::Vector3d &vsat_icf, double density,
                 double sat_mass) noexcept {
  /* normalized velocity vector */
  const auto vn = vsat_icf.normalized();

  /* add contributions from each macromodel plate */
  double A = 0e0;
  for (const auto &plate : macromodel_icf) {
    const double ct = vn.dot(plate.normal());
    if (ct < 0) {
      A += plate.area() * ct;
    }
  }

  Eigen::Vector3d acc =
      -0.5e0 * (A / sat_mass) * density * vsat_icf.squaredNorm() * vn;

  /* return total drag */
  return acc;
}
} /* namespace dso */

#endif
