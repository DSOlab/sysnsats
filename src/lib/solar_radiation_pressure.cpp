#include "iers/iersconst.hpp"
#include "srp.hpp"

/* Solar flux at 1 AU in [W/m^2]. */
constexpr const double P0 = 1367e0;

/* P0/c to be used for scaling SRP */
constexpr const double F_P0C = P0 / iers2010::C;

Eigen::Vector3d dso::solar_radiation_pressure(
    const std::vector<dso::MacromodelSurfaceElement> &macromodel,
    const Eigen::Vector3d &rsat, const Eigen::Vector3d &rsun, double mass,
    Eigen::Matrix3d &dadr) noexcept {
  dadr.setZero();

  const Eigen::Vector3d d = rsat - rsun;
  const double r2 = d.squaredNorm();
  const double r = std::sqrt(r2);

  const Eigen::Vector3d u = d / r;

  // f(u) = sum of plate contributions (old "acc" before scaling)
  auto sum_plates = [&](const Eigen::Vector3d &udir) -> Eigen::Vector3d {
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    for (const auto &plate : macromodel) {
      f += dso::plate_solar_radiation_pressure(udir, plate);
    }
    return f;
  };
  const Eigen::Vector3d f = sum_plates(u);

  // S = |rsun|^2 / |rsat-rsun|^2
  const double Rs2 = rsun.squaredNorm();
  const double S = Rs2 / r2;

  const double K = F_P0C / mass;

  // acceleration
  const Eigen::Vector3d a = K * S * f;

  // geometry derivatives
  // gradS is 1x3 row vector: dS/drsat
  const Eigen::RowVector3d gradS = (-2.0 * Rs2 / (r2 * r2)) * d.transpose();
  // du/drsat is 3x3
  const Eigen::Matrix3d du_dr =
      (Eigen::Matrix3d::Identity() - u * u.transpose()) / r;

  // df/du (3x3)
  Eigen::Matrix3d df_du = Eigen::Matrix3d::Zero();
  // If we want to IGNORE direction dependence, keep df_du = 0
  // (then dadr only includes inverse-square scaling term).
  {
    const double eps =
        1e-7; // dimensionless perturbation on unit vector components
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      e(j) = eps;

      // perturb then renormalize back to unit sphere
      const Eigen::Vector3d up = (u + e).normalized();
      const Eigen::Vector3d um = (u - e).normalized();

      const Eigen::Vector3d fp = sum_plates(up);
      const Eigen::Vector3d fm = sum_plates(um);

      df_du.col(j) = (fp - fm) / (2.0 * eps);
    }
  }

  // assemble dadr = da/drsat
  // term1: K * (f ⊗ gradS)
  const Eigen::Matrix3d term1 = K * (f * gradS); // (3x1)*(1x3) = 3x3

  // term2: K * S * (df/du)*(du/dr)
  const Eigen::Matrix3d term2 = K * S * (df_du * du_dr);

  dadr = term1 + term2;

  return a;
}

Eigen::Vector3d dso::solar_radiation_pressure(
    const std::vector<dso::MacromodelSurfaceElement> &macromodel,
    const Eigen::Vector3d &rsat, const Eigen::Vector3d &rsun,
    double Mass) noexcept {
  /* sun-to-satellite vector, normalized */
  const Eigen::Vector3d r = (rsat - rsun).normalized();

  /* add contributions from each macromodel plate */
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  for (const auto &plate : macromodel) {
    acc += dso::plate_solar_radiation_pressure(r, plate);
  }

  /* scaling factor for SRP */
  const double F_R0R = rsun.squaredNorm() / (rsat - rsun).squaredNorm();

  /* return SRP */
  return (acc / Mass) * (F_P0C * F_R0R);
}

Eigen::Vector3d dso::solar_radiation_pressure(double A,
                                              const Eigen::Vector3d &rsat,
                                              const Eigen::Vector3d &rsun,
                                              double Mass) noexcept {
  /* sun-to-satellite vector, normalized */
  const Eigen::Vector3d r = (rsat - rsun).normalized();

  /* scaling factor for SRP */
  const double F_R0R = rsun.squaredNorm() / (rsat - rsun).squaredNorm();

  // printf("SRP area = %.2f\n", A);

  /* return SRP acc = P * Cr * A * r_s*/
  return (F_P0C * F_R0R) * A * r / Mass;
}
