#ifndef __DSO_SATELLITE_OCCULTATION_MODELS_HPP__
#define __DSO_SATELLITE_OCCULTATION_MODELS_HPP__

#include "eigen3/Eigen/Eigen"
#include "iers/iersconst.hpp"

namespace dso
{
/** @brief Occulation factor according to the conical model.
 *
 * This model only consideres the Sun/Earth/satellite geometry, ignoring
 * atmosphere and oblateness of an occulting body.
 *
 * @param[in] sat Satellite coordinates in ECEF, Cartesian (x,y,z) in [m]
 * @param[in] sun Sun coordinates in ECEF, Cartesian (x,y,z) in [m]
 * @param[in] Re  Radius of the Earth [m]
 * @param[in] Rs  Radius of the Sun [m]
 *
 * Reference: Zhang et al, Study of satellite shadow function model considering
 * the overlapping parts of Earth shadow and Moon shadow and its application to
 * GPS satellite orbit determination, Advances in Space Research, Volume 63,
 * Issue 9, 2019, https://doi.org/10.1016/j.asr.2018.02.002.
 */
double conical_occultation(const Eigen::Vector3d &rsat,
                           const Eigen::Vector3d &rsun,
                           double Re = iers2010::Re,
                           double Rs = iers2010::Rs) noexcept;
} /* namespace dso */

#endif
