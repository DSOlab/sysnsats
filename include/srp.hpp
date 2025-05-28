#ifndef __DSO_SOLAR_RADIATION_PRESSURE_ACCELERATION_HPP__
#define __DSO_SOLAR_RADIATION_PRESSURE_ACCELERATION_HPP__

#include "eigen3/Eigen/Eigen"
#include "macromodel.hpp"

namespace dso {
Eigen::Vector3d plate_solar_radiation_pressure(
    const Eigen::Vector3d &sun_to_sat_normalized,
    const dso::MacromodelSurfaceElement &plate) noexcept;

Eigen::Vector3d solar_radiation_pressure(
    const std::vector<MacromodelSurfaceElement> &macromodel_icf,
    const Eigen::Vector3d &rsat_icf, const Eigen::Vector3d &rsun_icf, double sat_mass) noexcept;

/* 'Cannonball' model */
Eigen::Vector3d solar_radiation_pressure(double area,
                                         const Eigen::Vector3d &rsat_icf,
                                         const Eigen::Vector3d &rsun_icf,
                                         double sat_mass) noexcept;
} /* namespace dso */

#endif
