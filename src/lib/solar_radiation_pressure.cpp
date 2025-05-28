#include "iers/iersconst.hpp"
#include "srp.hpp"

/* Solar flux at 1 AU in [W/m^2]. */
constexpr const double P0 = 1367e0;

/* P0/c to be used for scaling SRP */
constexpr const double F_P0C = P0 / iers2010::C;

Eigen::Vector3d dso::solar_radiation_pressure(
    const std::vector<dso::MacromodelSurfaceElement> &macromodel,
    const Eigen::Vector3d &rsat, const Eigen::Vector3d &rsun, double Mass) noexcept {
  /* sun-to-satellite vector, normalized */
  const Eigen::Vector3d r = (rsat - rsun).normalized();

  /* add contributions from each macromodel plate */
  // printf("SRP area = ");
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  for (const auto &plate : macromodel) {
    acc += dso::plate_solar_radiation_pressure(r, plate);
  }
  // printf("\n");

  /* scaling factor for SRP */
  const double F_R0R = rsun.squaredNorm() / (rsat - rsun).squaredNorm();

  /* return SRP */
  return (acc/Mass) * (F_P0C * F_R0R);
}

Eigen::Vector3d dso::solar_radiation_pressure(
    double A,
    const Eigen::Vector3d &rsat, const Eigen::Vector3d &rsun, double Mass) noexcept {
  /* sun-to-satellite vector, normalized */
  const Eigen::Vector3d r = (rsat - rsun).normalized();
  
  /* scaling factor for SRP */
  const double F_R0R = rsun.squaredNorm() / (rsat - rsun).squaredNorm();

  // printf("SRP area = %.2f\n", A);

  /* return SRP acc = P * Cr * A * r_s*/
  return (F_P0C * F_R0R) * A * r / Mass;
}
