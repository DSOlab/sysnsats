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
      /* note: make this positive (ct is now <0) */
      A += plate.area() * (-ct);
    }
  }

  Eigen::Vector3d acc =
      -0.5e0 * (A / sat_mass) * density * vsat_icf.squaredNorm() * vn;

  /* return total drag */
  return acc;
}

Eigen::Vector3d atmospheric_drag(
    const std::vector<MacromodelSurfaceElement> &macromodel_icf,
    const Eigen::Vector3d &vsat_icf, double density, double sat_mass,
    const Eigen::Vector3d omega_gcrs,
    const Eigen::Vector3d &drhodr, /* [dρ/dx, dρ/dy, dρ/dz] as vector */
    Eigen::Matrix<double, 3, 3> &dadr,
    Eigen::Matrix<double, 3, 3> &dadv) noexcept {
  /* normalized velocity vector */
  const auto vn = vsat_icf.normalized();

  /* add contributions from each macromodel plate */
  double A = 0e0;
  for (const auto &plate : macromodel_icf) {
    const double ct = vn.dot(plate.normal());
    if (ct < 0) {
      /* note: make this positive (ct is now <0) */
      A += plate.area() * (-ct);
    }
  }

  const double beta = -0.5e0 * (A / sat_mass);
  const double V = vsat_icf.norm();

  Eigen::Vector3d acc = beta * density * vsat_icf.squaredNorm() * vn;

  /* gradient da/dr -> 3x3 */
  const auto S = V * Eigen::Matrix<double, 3, 3>::Identity() +
                 (vsat_icf * vsat_icf.transpose()) / V;
  Eigen::Matrix3d W;
  const auto w = omega_gcrs;
  W << 0e0, -w.z(), w.y(), w.z(), 0e0, -w.x(), -w.y(), w.x(), 0e0;
  dadr = beta * (V * vsat_icf * drhodr.transpose() - density * S * W);
  //  (1x1)*(3x1)*(1x3)                   (1x1)*(3x3)*(3x3)

  /* gradient da/dv -> 3x3 */
  dadv = beta * density * S;

  /* return total drag */
  return acc;
}

} /* namespace dso */

#endif
