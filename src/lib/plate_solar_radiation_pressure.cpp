#include "srp.hpp"

Eigen::Vector3d dso::plate_solar_radiation_pressure(
    /* normalized sun-to-satellite vector in ECI frame */
    const Eigen::Vector3d &r, const dso::MacromodelSurfaceElement &a) noexcept {
  /* cos(theta) between directions of surface normal and sun-satellite */
  const double ct = r.dot(a.normal());
  if (ct < 0) {
    return a.area() * ct *
           (2e0 * a.spec_optical() * ct * a.normal() +
            a.diff_optical() * (r - (2e0 / 3e0) * a.normal()) +
            a.abs_optical() * r);
  }
  return Eigen::Vector3d::Zero();
}