#ifndef __DSO_SATELLITE_INFO_FILE_HPP__
#define __DSO_SATELLITE_INFO_FILE_HPP__

#include "datetime/calendar.hpp"
#include "eigen3/Eigen/Eigen"

namespace dso {

enum class SATELLITE_SYSTEM { DORIS, SLR, GPS }; /* enum SATELLITE_SYSTEM */

template <SATELLITE_SYSTEM> struct SatelliteSystemTraits {};
template <SATELLITE_SYSTEM> struct SatelliteSystemObservationType {};

enum class SATELLITE { JASON1, JASON2, JASON3, SENTINEL3A, SENTINEL3B }; /* enum SATELLITE */

template <SATELLITE S> class SatelliteMacromodel {};

/** @brief Get satellite mass and CoG correction from a CNS mass file.
 *
 * These files are available at:
 * ftp://ftp.ids-doris.org/pub/ids/satellites/
 * as <SATID>mass.txt
 *
 * The file mass.readme contains information on the file format (under the
 * same ftp site).
 *
 * Given a date, this function will parse the satellite-specific file and
 * return the (commulative) correction that has to be applied at the initial
 * Mass and CoG to get the 'correct' mass and CoG (in the body-fixed ref.
 * frame) of the satellite at the epoch of request.
 *
 * @param[in] satmass_fn The filename of the satellite mass file.
 * @param[in] t          The epoch for which we want the corrections.
 * @param[out] dmass     (Commulative) mass correction, in [kg] to be applied
 *                       to the initial mass value.
 * @param[out] dxyz      (Commulative) offset correction, in [m] to be
 *                       applied to the initial CoG point, in body-fixed RF.
 * @return Anything other than zero denotes an error.
 */
int cnes_satellite_correction(const char *satmass_fn, MjdEpoch &t,
                              double &dmass,
                              Eigen::Matrix<double, 3, 1> &dxyz) noexcept;

} /* namespace dso */
#endif
